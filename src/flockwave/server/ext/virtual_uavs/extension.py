"""Extension that creates one or more virtual UAVs in the server.

REST API (mounted under ``/api/v1/virtual-uavs`` by default)
----------------------------------------------------------
GET  ``/``          current fleet status (enabled, count, UAV IDs)
POST ``/enable``    start (or keep) the virtual drone fleet
POST ``/disable``   stop the fleet and remove UAVs from the registry
POST ``/count``     set the number of virtual drones (body: ``{"count": N}``)

Enable/disable controls the simulated fleet while the extension stays loaded
so the API remains available. Changing the count rebuilds the fleet in place.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import ExitStack, contextmanager
from functools import partial
from logging import Logger
from random import uniform
from typing import TYPE_CHECKING, Any, Optional

from colour import Color
from flockwave.gps.vectors import (
    FlatEarthCoordinate,
    FlatEarthToGPSCoordinateTransformation,
)
from flockwave.spec.ids import make_valid_object_id
from quart import Blueprint, jsonify, request
from trio import CancelScope, Event, Lock, open_nursery, sleep, sleep_forever

from flockwave.server.registries.errors import RegistryFull

from ..base import UAVExtension
from .driver import VirtualUAV, VirtualUAVDriver
from .fw_upload import FIRMWARE_UPDATE_TARGET_ID
from .placement import place_drones

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

__all__ = ("construct", "dependencies", "description", "enhancers")

blueprint = Blueprint("virtual_uavs", __name__)

# Bound for the lifetime of the loaded extension (set in configure/teardown).
# Do not rely on run()-scoped `overridden()` alone: Quart blueprints cannot be
# unmounted, and HTTP requests may arrive before run() starts.
ext: Optional["VirtualUAVProviderExtension"] = None
app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None


def _require_ext() -> "VirtualUAVProviderExtension":
    if ext is not None:
        return ext

    # Fallback when module globals were cleared but the extension is still loaded
    # (e.g. blueprint left registered after run() exited).
    try:
        from flockwave.server.ext.http_server.extension import ext_manager

        if ext_manager is not None and ext_manager.is_loaded("virtual_uavs"):
            return ext_manager._get_loaded_extension_by_name("virtual_uavs")
    except Exception:
        pass

    raise RuntimeError(
        "Virtual UAV extension is not running. "
        "Set EXTENSIONS.virtual_uavs.enabled to true and restart the server."
    )


@blueprint.route("/", methods=["GET"])
async def status_endpoint():
    """Return the current virtual-drone fleet status."""
    try:
        return jsonify(_require_ext().fleet_status())
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503


@blueprint.route("/enable", methods=["POST"])
async def enable_endpoint():
    """Enable (start) the virtual drone fleet."""
    try:
        status = await _require_ext().set_fleet_enabled(True)
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503
    if log:
        log.info("Virtual UAV fleet enabled (%s drones)", status["count"])
    return jsonify({"success": True, **status})


@blueprint.route("/disable", methods=["POST"])
async def disable_endpoint():
    """Disable (stop) the virtual drone fleet."""
    try:
        status = await _require_ext().set_fleet_enabled(False)
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503
    if log:
        log.info("Virtual UAV fleet disabled")
    return jsonify({"success": True, **status})


@blueprint.route("/count", methods=["POST"])
async def count_endpoint():
    """Set the number of virtual drones.

    Request body (JSON)::

        {"count": 10}
    """
    body = await request.get_json(silent=True) or {}
    if "count" not in body:
        return jsonify({"error": "'count' is required"}), 400

    try:
        count = int(body["count"])
    except (TypeError, ValueError):
        return jsonify({"error": "'count' must be an integer"}), 400

    if count < 0:
        return jsonify({"error": "'count' must be >= 0"}), 400

    try:
        status = await _require_ext().set_fleet_count(count)
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400

    if log:
        log.info("Virtual UAV fleet count set to %s", count)
    return jsonify({"success": True, **status})


class VirtualUAVProviderExtension(UAVExtension[VirtualUAVDriver]):
    """Extension that creates one or more virtual UAVs in the server."""

    _driver: VirtualUAVDriver

    _delay: float = 1
    """Number of seconds that must pass between two consecutive
    simulated status updates to the UAVs.
    """

    uavs: list[VirtualUAV]
    """The list of virtual UAVs managed by this extension."""

    _configuration: dict[str, Any]
    """Last configuration dict used to build the fleet."""

    _fleet_enabled: bool
    """Whether the simulated fleet should be active."""

    _count: int
    """Desired number of virtual UAVs."""

    _fleet_lock: Lock
    """Serializes enable/disable/count updates against the worker loop."""

    _rebuild_event: Event | None
    """Set by API handlers to ask the worker to rebuild the fleet."""

    _rebuild_done: Event | None
    """Set by the worker when a requested rebuild has finished."""

    def __init__(self):
        """Constructor."""
        super().__init__()
        self.uavs = []
        self._configuration = {}
        self._fleet_enabled = True
        self._count = 0
        self._fleet_lock = Lock()
        self._rebuild_event = None
        self._rebuild_done = None

    def _create_driver(self):
        return VirtualUAVDriver()

    def configure(self, configuration):
        global ext, app, log

        super().configure(configuration)

        assert self.app is not None

        # Normalize deprecated key before we keep a reference.
        if "origin" not in configuration and "center" in configuration:
            if self.log:
                self.log.warning("'center' is deprecated; use 'origin' instead")
            configuration["origin"] = configuration.pop("center")

        self._configuration = configuration
        self.delay = configuration.get("delay", 1)
        self._count = max(int(configuration.get("count", 0)), 0)
        # `active` controls the simulated fleet; extension load still uses
        # the top-level `enabled` flag understood by the extension manager.
        self._fleet_enabled = bool(configuration.get("active", True))
        if self._fleet_enabled:
            self._rebuild_uavs()
        else:
            self.uavs = []

        # Expose this instance to REST handlers as soon as the extension loads.
        ext = self
        app = self.app
        log = self.log

    def teardown(self) -> None:
        global ext, app, log

        if ext is self:
            ext = None
            app = None
            log = None
        super().teardown()

    def configure_driver(self, driver: VirtualUAVDriver, configuration):
        # Set whether the virtual drones should be armed after boot
        driver.uavs_armed_after_boot = bool(configuration.get("arm_after_boot"))
        driver.use_battery_percentages = bool(
            configuration.get("use_battery_percentages", True)
        )
        driver.battery_auto_recharging = bool(
            configuration.get("battery_auto_recharging", True)
        )

    def _rebuild_uavs(self) -> None:
        """(Re)create the virtual UAV objects from the current configuration."""
        assert self.app is not None
        assert self._driver is not None

        configuration = self._configuration
        count = self._count
        id_format = configuration.get("id_format", "VIRT-{0}")
        default_takeoff_area = {"type": "grid", "spacing": 5}

        origin = configuration.get("origin")
        if not origin:
            raise ValueError("virtual_uavs configuration requires an 'origin'")

        origin_amsl = origin[2] if len(origin) > 2 else None
        coordinate_system = {
            "origin": origin[:2],
            "orientation": configuration.get("orientation", 0),
            "type": configuration.get("type", "nwu"),
        }
        trans = FlatEarthToGPSCoordinateTransformation.from_json(coordinate_system)

        home_positions = [
            FlatEarthCoordinate(x=vec.x, y=vec.y, amsl=origin_amsl, ahl=0)
            for vec in place_drones(
                count, **configuration.get("takeoff_area", default_takeoff_area)
            )
        ]

        if configuration.get("add_noise", False):
            position_noise = 0.2
            heading_noise = 3
            home_positions = [
                FlatEarthCoordinate(
                    x=p.x + uniform(-position_noise, position_noise),
                    y=p.y + uniform(-position_noise, position_noise),
                    # TODO: add amsl noise if we can be sure that amsl is not None
                    amsl=p.amsl,  # + uniform(-position_noise, position_noise),
                    ahl=p.ahl,
                )
                for p in home_positions
            ]
            headings = [
                (trans.orientation + uniform(-heading_noise, heading_noise)) % 360
                for _p in home_positions
            ]
        else:
            headings = [trans.orientation] * len(home_positions)

        uav_ids = [
            make_valid_object_id(id_format.format(index)) for index in range(count)
        ]
        self.uavs = [
            self._driver.create_uav(id, home=trans.to_gps(home), heading=heading)
            for id, home, heading in zip(uav_ids, home_positions, headings)
        ]

        try:
            radiation_ext = self.app.extension_manager.import_api("radiation")
        except Exception:
            radiation_ext = None
        for uav in self.uavs:
            uav.radiation_ext = radiation_ext

    def fleet_status(self) -> dict[str, Any]:
        """Return a JSON-serializable status snapshot of the fleet."""
        return {
            "enabled": self._fleet_enabled,
            "count": self._count,
            "uavIds": [uav.id for uav in self.uavs] if self._fleet_enabled else [],
        }

    async def set_fleet_enabled(self, enabled: bool) -> dict[str, Any]:
        """Enable or disable the simulated fleet.

        Args:
            enabled: whether the fleet should be active

        Returns:
            updated fleet status
        """
        return await self._update_fleet(enabled=enabled)

    async def set_fleet_count(self, count: int) -> dict[str, Any]:
        """Set the number of virtual drones and rebuild the fleet if needed.

        Args:
            count: desired number of virtual UAVs (``>= 0``)

        Returns:
            updated fleet status
        """
        if count < 0:
            raise ValueError("count must be >= 0")
        return await self._update_fleet(count=count)

    async def _update_fleet(
        self, *, enabled: bool | None = None, count: int | None = None
    ) -> dict[str, Any]:
        """Apply fleet changes and restart the worker nursery when it is running."""
        async with self._fleet_lock:
            if enabled is not None:
                self._fleet_enabled = bool(enabled)
                self._configuration["active"] = self._fleet_enabled
            if count is not None:
                self._count = int(count)
                self._configuration["count"] = self._count

            if self._rebuild_event is not None:
                done = Event()
                self._rebuild_done = done
                self._rebuild_event.set()
                await done.wait()
            else:
                # Worker is not running (no clients connected); just refresh
                # the in-memory UAV list for the next spin-up.
                if self._fleet_enabled:
                    self._rebuild_uavs()
                else:
                    self.uavs = []

            return self.fleet_status()

    @property
    def delay(self):
        """Number of seconds that must pass between two consecutive
        simulated status updates to the UAVs.
        """
        return self._delay

    @delay.setter
    def delay(self, value):
        self._delay = max(float(value), 0)

    async def simulate_uav(self, uav: VirtualUAV, spawn: Callable):
        """Simulates the behaviour of a single UAV in the application.

        Parameters:
            uav: the virtual UAV to simulate
            spawn: function to call when the UAV wishes to spawn a background
                task
        """
        assert self.app is not None

        try:
            await self._simulate_uav(uav, spawn)
        except RegistryFull:
            self.app.handle_registry_full_error(self, "simulated UAV")

    async def _simulate_uav(self, uav: VirtualUAV, spawn: Callable):
        assert self.app is not None

        updater = partial(self.app.request_to_send_UAV_INF_message_for, [uav.id])

        with self.app.object_registry.use(uav):
            while True:
                # Simulate the UAV behaviour from boot time
                shutdown_reason = await uav.run_single_boot(
                    self._delay,
                    mutate=self.create_device_tree_mutation_context,
                    notify=updater,
                    spawn=spawn,
                )

                # If we need to restart, let's restart after a short delay.
                # Otherwise let's stop the loop.
                if shutdown_reason == "shutdown":
                    break
                else:
                    await sleep(0.2)

    async def run(self):
        global ext, app, log

        assert self.app is not None

        # Keep REST handlers bound even if a previous run() context exited
        # while the Quart blueprint remained registered.
        ext = self
        app = self.app
        log = self.log

        route = self._configuration.get("route", "/api/v1/virtual-uavs")
        http_server = self.app.import_api("http_server")
        signals = self.app.import_api("signals")

        with ExitStack() as stack:
            stack.enter_context(http_server.mounted(blueprint, path=route))
            stack.enter_context(
                signals.use({"show:lights_updated": self._on_lights_updated})
            )
            if self.log:
                self.log.info(
                    "Virtual UAV API mounted at %s "
                    "(GET /, POST /enable, POST /disable, POST /count)",
                    route,
                )
            await sleep_forever()

    @staticmethod
    @contextmanager
    def use_firmware_update_support(api) -> Iterator[None]:
        """Enhancer context manager that adds support for remote firmware updates
        to virtual UAVs.
        """
        target = api.create_target(
            id=FIRMWARE_UPDATE_TARGET_ID, name="Virtual UAV firmware"
        )
        with api.use_target(target):
            yield

    async def worker(self, app, configuration, logger):
        """Main background task of the extension that updates the state of
        the UAVs periodically.

        The worker loop can be restarted in-place when the REST API changes
        the fleet size or enable flag.
        """
        try:
            while True:
                self._rebuild_event = Event()
                rebuild_done = self._rebuild_done

                if self._fleet_enabled:
                    self._rebuild_uavs()
                else:
                    self.uavs = []

                with CancelScope() as scope:
                    async with open_nursery() as nursery:
                        if self._fleet_enabled:
                            for uav in self.uavs:
                                nursery.start_soon(
                                    self.simulate_uav, uav, nursery.start_soon
                                )
                        # Tell any waiting API handler that the new fleet is up.
                        if rebuild_done is not None:
                            rebuild_done.set()
                            self._rebuild_done = None
                        await self._rebuild_event.wait()
                        scope.cancel()
        finally:
            self._rebuild_event = None
            if self._rebuild_done is not None:
                self._rebuild_done.set()
                self._rebuild_done = None

    def _on_lights_updated(self, sender, config):
        color = config.color if str(config.effect.value) == "solid" else None
        if color is not None:
            color = Color(rgb=(x / 255.0 for x in color))

        for uav in self.uavs:
            uav.set_led_color(color)


construct = VirtualUAVProviderExtension
dependencies = ("http_server", "signals")
description = "Simulated, non-realistic UAVs for testing or demonstration purposes"
enhancers = {"firmware_update": VirtualUAVProviderExtension.use_firmware_update_support}
