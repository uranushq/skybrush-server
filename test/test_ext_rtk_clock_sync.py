from datetime import datetime, timezone
from struct import pack

from flockwave.gps.ubx.enums import UBXClass, UBXNAVSubclass
from flockwave.gps.ubx.packet import UBXPacket

import flockwave.server.ext.rtk.clock_sync as mod
from flockwave.server.ext.rtk.clock_sync import (
    RESYNC_THRESHOLD_SECONDS,
    SYNC_THRESHOLD_SECONDS,
    GPSClockSynchronizationValidator,
)


def _timeutc_packet(
    dt: datetime, *, nano: int = 0, valid_utc: bool = True
) -> UBXPacket:
    """Build a minimal UBX-NAV-TIMEUTC packet for the given UTC datetime."""
    valid = 0x04 if valid_utc else 0x00
    payload = pack(
        "<IIiH5BB",
        0,  # iTOW
        0,  # tAcc
        nano,
        dt.year,
        dt.month,
        dt.day,
        dt.hour,
        dt.minute,
        dt.second,
        valid,
    )
    return UBXPacket(int(UBXClass.NAV), int(UBXNAVSubclass.TIMEUTC), payload)


def _freeze_now(monkeypatch, now: datetime) -> None:
    class FakeDateTime(datetime):
        @classmethod
        def now(cls, tz=None):
            return now

    monkeypatch.setattr(mod, "datetime", FakeDateTime)


def test_in_sync_when_within_two_seconds(monkeypatch):
    now = datetime(2026, 8, 13, 0, 0, 10, tzinfo=timezone.utc)
    _freeze_now(monkeypatch, now)

    validator = GPSClockSynchronizationValidator()
    validator.notify(_timeutc_packet(now.replace(microsecond=0), nano=500_000_000))
    assert validator.are_clocks_in_sync is True


def test_out_of_sync_when_gps_ahead_or_behind(monkeypatch):
    now = datetime(2026, 8, 13, 0, 0, 10, tzinfo=timezone.utc)
    _freeze_now(monkeypatch, now)

    validator = GPSClockSynchronizationValidator()

    # GPS 3 seconds ahead
    validator.notify(_timeutc_packet(now.replace(second=13)))
    assert validator.are_clocks_in_sync is False

    # GPS 3 seconds behind (old one-sided check missed this)
    validator.assume_sync()
    validator.notify(_timeutc_packet(now.replace(second=7)))
    assert validator.are_clocks_in_sync is False


def test_hysteresis_requires_tighter_error_to_resync(monkeypatch):
    now = datetime(2026, 8, 13, 0, 0, 10, tzinfo=timezone.utc)
    _freeze_now(monkeypatch, now)

    validator = GPSClockSynchronizationValidator()
    validator.notify(_timeutc_packet(now.replace(second=7)))  # |delta|=3
    assert validator.are_clocks_in_sync is False

    # |delta| = 1.8: below SYNC (2.0) but above RESYNC (1.5) → stay out
    assert RESYNC_THRESHOLD_SECONDS < 1.8 < SYNC_THRESHOLD_SECONDS
    validator.notify(_timeutc_packet(now.replace(second=8), nano=200_000_000))
    assert validator.are_clocks_in_sync is False

    # |delta| = 1.0 → resync
    validator.notify(_timeutc_packet(now.replace(second=9)))
    assert validator.are_clocks_in_sync is True


def test_invalid_utc_flag_is_ignored():
    validator = GPSClockSynchronizationValidator()
    events: list[bool] = []
    validator.sync_state_changed.connect(lambda sender, in_sync: events.append(in_sync))

    now = datetime(2026, 8, 13, 0, 0, 10, tzinfo=timezone.utc)
    validator.notify(_timeutc_packet(now.replace(second=0), valid_utc=False))
    assert validator.are_clocks_in_sync is True
    assert events == []
