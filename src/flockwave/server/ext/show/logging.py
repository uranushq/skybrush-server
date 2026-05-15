"""Logging middleware that the show extension installs."""

from logging import Logger
from time import monotonic
from typing import Any, cast

from flockwave.gps.vectors import FlatEarthToGPSCoordinateTransformation

from flockwave.server.model import Client, FlockwaveMessage
from flockwave.server.show import ShowSpecification
from flockwave.server.utils import format_uav_ids_nicely

from .metadata import ShowMetadata

ShowFingerprint = list[Any]
"""Typing specification for the fingerprint of a show containing its most
basic parameters.
"""


def get(input: dict[str, Any], *args: str) -> Any:
    """Helper function to retrieve deeply nested items from a dict-of-dicts."""
    result: Any = input
    for arg in args:
        if isinstance(result, dict):
            result = result.get(arg)
        else:
            result = None
            break
    return result


class ShowUploadLoggingMiddleware:
    """Logging middleware that the show extension installs. It will print log
    messages whenever it detects that a new show upload has started.
    """

    _last_show_metadata: ShowMetadata | None = None
    """The metadata of the last show upload that was seen by the
    middleware.
    """

    _last_show_upload_command_at: float
    """Timestamp when the last show upload command was detected."""

    _last_show_upload_fingerprint: ShowFingerprint | None = None
    """Fingerprint containing the basic parameters of the last show upload.
    Used to decide whether it's a new show upload or most likely not.
    """

    _last_show_upload_uav_ids: tuple[str, ...] = ()
    """UAV IDs that were targeted by the last show upload."""

    _pending_show_uploads: dict[str, tuple[str, str]]
    """Mapping from async receipt IDs to UAV IDs and show IDs."""

    _log: Logger
    """Logger that the middleware will write to."""

    def __init__(self, log: Logger):
        """Constructor.

        Parameter:
            log: logger that the middleware will write to
        """
        self._last_show_upload_command_at = monotonic() - 1000
        self._pending_show_uploads = {}
        self._log = log

    def __call__(self, message: FlockwaveMessage, sender: Client) -> FlockwaveMessage:
        show = self._extract_show(message)
        if show:
            now = monotonic()
            fingerprint = self._get_show_fingerprint(show)
            uav_ids = tuple(message.get_ids())

            should_log = (
                fingerprint != self._last_show_upload_fingerprint
                or uav_ids != self._last_show_upload_uav_ids
                or now - self._last_show_upload_command_at >= 30
            )
            if should_log:
                fmt_fingerprint = self._format_fingerprint(fingerprint)
                show_id = str(fingerprint[0]) if fingerprint[0] else ""
                sep = ", " if fmt_fingerprint else ""
                self._log.info(
                    f"Show upload started for {format_uav_ids_nicely(uav_ids)}"
                    f"{sep}{fmt_fingerprint}",
                    extra={"id": show_id},
                )

                self._last_show_metadata = self._get_metadata_from_upload_request(show)

            self._last_show_upload_command_at = now
            self._last_show_upload_fingerprint = fingerprint
            self._last_show_upload_uav_ids = uav_ids

        return message

    def log_response(
        self,
        message: FlockwaveMessage,
        to: Client | None,
        in_response_to: FlockwaveMessage | None,
    ) -> FlockwaveMessage:
        """Logs the outcome of a show upload command based on its response."""
        if in_response_to is None:
            self._log_async_response(message)
            return message

        show = self._extract_show(in_response_to)
        if not show:
            return message

        target_ids = tuple(in_response_to.get_ids())
        successful_ids = self._get_status_ids(message, "result", "success")
        receipts = self._get_receipts(message)
        pending_ids = set(receipts)
        error_reasons = self._get_error_reasons(message)
        failed_ids = set(error_reasons)

        succeeded = tuple(
            uav_id for uav_id in target_ids if uav_id in successful_ids - failed_ids
        )
        pending = tuple(uav_id for uav_id in target_ids if uav_id in pending_ids)
        failed = tuple(uav_id for uav_id in target_ids if uav_id in failed_ids)

        if not (succeeded or pending or failed):
            return message

        fingerprint = self._get_show_fingerprint(show)
        show_id = str(fingerprint[0]) if fingerprint[0] else ""

        for uav_id, receipt_id in receipts.items():
            if uav_id in target_ids:
                self._pending_show_uploads[receipt_id] = (uav_id, show_id)

        if failed:
            status_parts = []
            if succeeded:
                status_parts.append(f"succeeded for {format_uav_ids_nicely(succeeded)}")
            if pending:
                status_parts.append(f"pending for {format_uav_ids_nicely(pending)}")
            status_parts.append(f"failed for {format_uav_ids_nicely(failed)}")
            details = self._format_error_reasons(error_reasons, failed)
            sep = ": " if details else ""
            self._log.warning(
                f"Show upload finished: {', '.join(status_parts)}{sep}{details}",
                extra={"id": show_id},
            )
        elif pending:
            self._log.info(
                f"Show upload accepted for {format_uav_ids_nicely(pending)}",
                extra={"id": show_id},
            )
        else:
            self._log.info(
                f"Show upload succeeded for {format_uav_ids_nicely(succeeded)}",
                extra={"id": show_id},
            )

        return message

    def _log_async_response(self, message: FlockwaveMessage) -> None:
        """Logs the final outcome of an async show upload command."""
        if message.get_type() != "ASYNC-RESP":
            return

        receipt_id = message.body.get("id")
        if not isinstance(receipt_id, str):
            return

        pending_upload = self._pending_show_uploads.pop(receipt_id, None)
        if pending_upload is None:
            return

        uav_id, show_id = pending_upload
        error = message.body.get("error")
        if error:
            self._log.warning(
                f"Show upload failed for {format_uav_ids_nicely((uav_id,))}: {error}",
                extra={"id": show_id},
            )
        else:
            self._log.info(
                f"Show upload succeeded for {format_uav_ids_nicely((uav_id,))}",
                extra={"id": show_id},
            )

    def _extract_show(self, message: FlockwaveMessage) -> ShowSpecification | None:
        """Checks whether the given message is a show upload and extracts the
        show specification out of the message if it is.
        """
        type = message.get_type()
        if type == "OBJ-CMD":
            cmd = message.body.get("command", "")
            if cmd == "__show_upload":
                kwds = message.body.get("kwds", {})
                if isinstance(kwds, dict) and "show" in kwds:
                    return kwds["show"]

    @staticmethod
    def _format_error_reasons(
        error_reasons: dict[str, str], failed_ids: tuple[str, ...]
    ) -> str:
        """Formats per-UAV upload errors for logging purposes."""
        max_items = 3
        parts = [
            f"{uav_id}: {error_reasons[uav_id]}"
            for uav_id in failed_ids[:max_items]
            if error_reasons.get(uav_id)
        ]
        excess = len(failed_ids) - max_items
        if excess > 0:
            parts.append(f"{excess} more")
        return "; ".join(parts)

    @staticmethod
    def _get_error_reasons(message: FlockwaveMessage) -> dict[str, str]:
        """Returns the error mapping from a response body."""
        errors = message.body.get("error")
        if isinstance(errors, dict):
            return {str(key): str(value) for key, value in errors.items()}
        return {}

    @staticmethod
    def _get_receipts(message: FlockwaveMessage) -> dict[str, str]:
        """Returns the UAV ID to async receipt ID mapping from a response body."""
        receipts = message.body.get("receipt")
        if isinstance(receipts, dict):
            return {str(key): str(value) for key, value in receipts.items()}
        return {}

    @staticmethod
    def _get_status_ids(message: FlockwaveMessage, *keys: str) -> set[str]:
        """Returns UAV IDs listed under response status keys."""
        result: set[str] = set()
        for key in keys:
            value = message.body.get(key)
            if isinstance(value, dict):
                result.update(str(key) for key in value)
            elif isinstance(value, list):
                result.update(str(item) for item in value)
        return result

    @property
    def last_show_metadata(self) -> ShowMetadata | None:
        """Returns the metadata of the last show upload that was seen by the
        middleware.
        """
        return self._last_show_metadata

    @staticmethod
    def _get_show_fingerprint(show: ShowSpecification) -> ShowFingerprint:
        """Extracts the basic show parameters like the origin and the orientation
        from the upload. These are used to decide whether an upload attempt is
        probably a continuation of an ongoing sequence of requests from the
        client or a completely new one.
        """
        show_dict = cast(dict[str, Any], show)
        return [
            get(show_dict, "mission", "id"),
            get(show_dict, "coordinateSystem"),
            get(show_dict, "amslReference"),
        ]

    @staticmethod
    def _get_metadata_from_upload_request(show: ShowSpecification) -> ShowMetadata:
        """Extracts the metadata of the current show being uploaded. This is
        returned to consumers of the API of the show extension when the caller
        requests the metadata of the last uploaded show.
        """
        coordinate_system = show.get("coordinateSystem")
        if not isinstance(coordinate_system, dict):
            coordinate_system = {}

        geofence = show.get("geofence")
        if not isinstance(geofence, dict):
            geofence = None

        mission = show.get("mission")
        if not isinstance(mission, dict):
            mission = {}

        maybe_amsl_reference = show.get("amslReference")
        if not isinstance(maybe_amsl_reference, (int, float)):
            maybe_amsl_reference = None

        return {
            "coordinateSystem": {
                "origin": coordinate_system.get("origin"),
                "orientation": str(coordinate_system.get("orientation", "")),
                "type": coordinate_system.get("type"),
            },
            "amslReference": maybe_amsl_reference,
            "geofence": geofence,
            "mission": {
                "id": str(mission.get("id", "")),
                "title": str(mission.get("title", "")),
                "numDrones": int(mission.get("numDrones", 0)),
            },
        }

    @staticmethod
    def _format_fingerprint(fingerprint: ShowFingerprint) -> str:
        """Returns a formatted representation of the show fingerprint for
        logging purposes.
        """
        parts = []
        if fingerprint[1] and isinstance(fingerprint[1], dict):
            try:
                xform = FlatEarthToGPSCoordinateTransformation.from_json(fingerprint[1])
            except (TypeError, RuntimeError):
                # TypeError may be raised if fingerprint[1]["origin"] is None,
                # which may be the case for indoor shows
                xform = None
            if xform:
                try:
                    parts.append(
                        f"{xform.orientation:.1f}° {xform.type.upper()} "
                        f"at {xform.origin.lat:.9g}° {xform.origin.lon:.9g}°"
                    )
                except Exception:
                    pass
        if fingerprint[2] is not None:
            try:
                parts.append(f"AMSL reference at {fingerprint[2]:.1f}m")
            except Exception:
                pass
        return ", ".join(parts)
