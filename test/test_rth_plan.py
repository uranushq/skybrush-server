"""Tests for RTH plan encoding."""

from __future__ import annotations

from flockwave.server.show.rth_plan import (
    RTHAction,
    RTHPlan,
    RTHPlanEntry,
    encode_rth_plan_block,
    encode_rth_plan_from_show,
)


def test_encode_rth_plan_block_roundtrip() -> None:
    payload = encode_rth_plan_block(
        {
            "version": 1,
            "entries": [
                {"time": 0, "action": "land"},
                {
                    "time": 5,
                    "action": "goTo",
                    "target": [0.0, 0.0],
                    "duration": 10,
                },
                {"time": 20, "action": "land"},
            ],
        }
    )
    assert payload is not None
    assert payload[0] == 1
    assert len(payload) > 4


def test_encode_rth_plan_from_show() -> None:
    show = {
        "rthPlan": {
            "version": 1,
            "entries": [
                {
                    "time": 0,
                    "action": "goTo",
                    "target": [1.5, -2.0],
                    "duration": 3,
                },
                {"time": 5, "action": "land"},
            ],
        }
    }
    payload = encode_rth_plan_from_show(show)
    assert payload is not None


def test_rth_plan_entry_json_conversion() -> None:
    entry = RTHPlanEntry(
        time=5,
        action=RTHAction.GO_TO_KEEPING_ALTITUDE_AND_LAND,
        target=(20.0, 20.0),
        duration=10,
    )
    restored = RTHPlanEntry.from_json(entry.to_json())
    assert restored == entry


def test_rth_plan_requires_increasing_timestamps() -> None:
    plan = RTHPlan()
    plan.add_entry(RTHPlanEntry(time=0, action=RTHAction.LAND))
    plan.add_entry(
        RTHPlanEntry(
            time=5,
            action=RTHAction.GO_TO_KEEPING_ALTITUDE_AND_LAND,
            target=(1.0, 2.0),
            duration=3,
        )
    )
    assert len(plan) == 2
