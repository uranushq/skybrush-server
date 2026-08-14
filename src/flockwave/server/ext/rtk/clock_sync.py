"""Class that validates whether the clock of the GPS is in sync with the
clock of the computer running the server.
"""

from datetime import datetime, timedelta, timezone
from struct import Struct
from typing import ClassVar

from blinker import Signal
from flockwave.gps.ubx.enums import UBXClass, UBXNAVSubclass
from flockwave.gps.ubx.packet import UBXPacket

from .types import GPSPacket

__all__ = ("GPSClockSynchronizationValidator",)

# Clocks are considered in sync when |GPS − server| is below this threshold.
# UBX-NAV-TIMEUTC seconds are integer; including ``nano`` still leaves room for
# transport delay and mild PC clock drift, so 2 s is used instead of 1 s.
SYNC_THRESHOLD_SECONDS = 2.0

# Hysteresis: once out of sync, require a tighter error before declaring sync
# again so 1 Hz TIMEUTC packets do not toggle the warning every second.
RESYNC_THRESHOLD_SECONDS = 1.5


class GPSClockSynchronizationValidator:
    """Class that validates whether the clock of the GPS is in sync with the
    clock of the computer running the server.
    """

    _are_clocks_in_sync: bool = True
    """Stores whether the server clock is assumed to be in sync with the GPS
    clock.
    """

    # iTOW(U4) + tAcc(U4) skipped, then nano(I4), year(U2), month..sec (5×U1)
    _ubx_nav_timeutc_struct: ClassVar[Struct] = Struct("<8xiH5B")

    sync_state_changed: Signal = Signal(
        doc=(
            "Signal sent whenever the validator detects that the GPS clock is "
            "out of sync with the server, or whenever sync is restored."
        )
    )

    @property
    def are_clocks_in_sync(self) -> bool:
        """Returns whether the server clock is assumed to be in sync with the GPS
        clock.
        """
        return self._are_clocks_in_sync

    @are_clocks_in_sync.setter
    def are_clocks_in_sync(self, value: bool) -> None:
        if bool(value) == self._are_clocks_in_sync:
            return

        self._are_clocks_in_sync = bool(value)
        self.sync_state_changed.send(self, in_sync=self._are_clocks_in_sync)

    def assume_sync(self) -> None:
        """Forces the validator to assume that the server clock and the GPS
        clock are in sync.
        """
        self.are_clocks_in_sync = True

    def notify(self, packet: GPSPacket) -> None:
        """Notifies the clock synchronization validator about the arrival of a
        new packet from the GPS.
        """
        if (
            isinstance(packet, UBXPacket)
            and packet.class_id == UBXClass.NAV
            and packet.subclass_id == UBXNAVSubclass.TIMEUTC
        ):
            self._handle_ubx_nav_timeutc(packet)

    def _handle_ubx_nav_timeutc(self, packet: UBXPacket) -> None:
        """Handles a NAV-TIMEUTC packet by parsing the UTC datetime and checking
        whether it matches the current date/time on the computer running the
        server.
        """
        payload = packet.payload
        if len(payload) < 20:
            # Invalid or short packet
            return

        if payload[19] & 0x04 != 0x04:
            # Packet does not contain a valid UTC timestamp
            return

        try:
            struct = self._ubx_nav_timeutc_struct
            nano, year, month, day, hour, minute, second = struct.unpack(
                payload[: struct.size]
            )
            dt = datetime(
                year, month, day, hour, minute, second, tzinfo=timezone.utc
            ) + timedelta(microseconds=nano // 1000)
            delta_seconds = abs((dt - datetime.now(timezone.utc)).total_seconds())
        except Exception:
            return

        if self._are_clocks_in_sync:
            self.are_clocks_in_sync = delta_seconds < SYNC_THRESHOLD_SECONDS
        else:
            self.are_clocks_in_sync = delta_seconds < RESYNC_THRESHOLD_SECONDS
