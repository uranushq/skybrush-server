"""Convert path-planner algorithm output → Skybrush show files.

This module takes a ``SolverResult`` (per-drone waypoint lists produced by
the greedy path-planning solver) and produces:

1. **Per-drone Skybrush trajectory specification dicts** – the JSON structure
   that ``TrajectorySpecification`` expects (version 1, linear or cubic
   Bézier segments).
2. **Per-drone ``.skyb`` binary show files** – the compact binary format that
   can be uploaded to MAVLink drones.
3. A **combined show JSON file** containing all drone trajectories.

Safety contract
---------------
The solver proves clearance for *synchronized, constant-speed linear* motion
using envelopes inflated by ``PLANNING_MARGIN``. Everything this module does
to the timing must keep each drone within a bounded distance of that nominal
schedule:

- Velocity smoothing eases the speed **per segment only** for solver output
  (no collinear-run merging), which bounds the schedule deviation to at most
  ~9.7% of a single solver step — far below the planning margin.
- Peak speeds of eased segments are computed in closed form and clamped to
  ``MAX_VELOCITY_XY`` / ``MAX_VELOCITY_Z``; if a trajectory cannot be eased
  within the limits it falls back to constant-speed motion, and if even the
  cruise speed violates the limits a :class:`TrajectoryLimitError` is raised
  so the request fails loudly instead of producing an unflyable show.
- Takeoff/landing durations are sized so the **peak** vertical speed (not the
  average) equals the configured takeoff/landing speed.

The final spatio-temporal verification gate (see ``verify``) re-checks the
built trajectories independently of these guarantees.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .solver import SolverResult

# Single source of truth for the per-step duration default. 1000 ms per
# 1 m solver step gives a 1 m/s cruise speed; with the default easing the
# peak speed at ramp segments is 1.5 m/s — far below MAX_VELOCITY_XY, and
# the per-segment peak clamp enforces the limits regardless of what a
# request chooses here.
DEFAULT_DURATION_MS = 1000

# Hard velocity limits enforced on every generated trajectory (m/s). These
# are also exported into the .skyc validation block so the planner, the
# firmware expectations and the Viewer validation agree on one set of values.
MAX_VELOCITY_XY = 8.0
MAX_VELOCITY_Z = 2.5

# Conservative default for small quadrotors during in-place turns.
DEFAULT_MAX_YAW_RATE_DEG_S = 90.0

# Default horizontal cruise speed during formation moves (m/s). Keep in
# sync with DEFAULT_DURATION_MS (= step_size / cruise speed for a 1 m step).
DEFAULT_CRUISE_SPEED_M_S = 1.0

# Default vertical speed while descending to land (m/s).
DEFAULT_LANDING_SPEED_M_S = 0.2

# Default vertical speed during takeoff (m/s).
DEFAULT_TAKEOFF_SPEED_M_S = 1.5

# Default velocity-smoothing strength applied to every generated path. See
# ``apply_velocity_smoothing`` for the exact meaning. This is the value used
# when the path-planner extension is loaded without an explicit configuration
# override; the server config UI can adjust it globally.
DEFAULT_VELOCITY_SMOOTHING = 1.0

# Angle (degrees) between the incoming and outgoing direction above which a
# waypoint is treated as a direction-change "corner". At a corner the geometry
# is a straight line on both sides, so any non-zero speed forces an
# instantaneous change of the velocity *vector* (a lateral jerk); we therefore
# ramp the speed down there. Below this threshold the waypoint is treated as a
# near-collinear pass-through and cruise speed is preserved.
CORNER_ANGLE_THRESHOLD_DEG = 5.0

# Ease makes the peak speed of a start-from-rest/stop-to-rest segment 1.5×
# its average speed (property of the cubic ease profile).
_EASE_PEAK_FACTOR = 1.5


class TrajectoryLimitError(ValueError):
    """A generated trajectory cannot satisfy the velocity or yaw-rate limits."""


class YawRateError(TrajectoryLimitError):
    """A yaw change does not fit its time budget at the allowed yaw rate."""


def duration_ms_for_cruise_speed(step_size: float, cruise_speed_m_s: float) -> int:
    """Return milliseconds per solver step for a target cruise speed."""
    if step_size <= 0:
        raise ValueError("step_size must be positive")
    if cruise_speed_m_s <= 0:
        raise ValueError("cruise_speed must be positive")
    return max(1, round(step_size / cruise_speed_m_s * 1000))


def step_time_ms(rec, duration_ms: int) -> int:
    """Absolute solver-timeline time for *rec* in milliseconds.

    Prefers ``rec.time_ms`` when present (variable per-segment timing);
    otherwise falls back to the uniform ``step * duration_ms`` schedule.
    """
    if getattr(rec, "time_ms", None) is not None:
        return int(rec.time_ms)
    return int(rec.step) * int(duration_ms)


def step_time_sec(rec, duration_ms: int) -> float:
    """Absolute solver-timeline time for *rec* in seconds."""
    return step_time_ms(rec, duration_ms) / 1000.0


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------


def _vertical_segment_duration(
    altitude_delta: float, speed: float, smoothing: float
) -> float:
    """Duration of a vertical climb/descent so its *peak* speed equals *speed*.

    With easing enabled the segment starts and ends at rest, which makes the
    peak speed 1.5× the average — so the segment must take 1.5× longer for
    the same peak. Without easing the motion is constant-speed.
    """
    if altitude_delta <= 0:
        return 0.0
    factor = _EASE_PEAK_FACTOR if smoothing > 0 else 1.0
    return round(factor * altitude_delta / speed, 4)


def _takeoff_landing_profile(
    first_pos: Sequence[float],
    last_pos: Sequence[float],
    ground_start_z: float,
    ground_end_z: float,
    takeoff_speed: float,
    landing_speed: float,
    smoothing: float,
) -> tuple[float, float, float, float]:
    """Shared takeoff/landing math for the trajectory and yaw builders.

    Returns ``(takeoff_alt, takeoff_duration, landing_alt, landing_duration)``.
    """
    takeoff_alt = max(0.0, first_pos[2] - ground_start_z)
    landing_alt = max(0.0, last_pos[2] - ground_end_z)
    return (
        takeoff_alt,
        _vertical_segment_duration(takeoff_alt, takeoff_speed, smoothing),
        landing_alt,
        _vertical_segment_duration(landing_alt, landing_speed, smoothing),
    )


def solver_result_to_trajectory_dicts(
    result: SolverResult,
    duration_ms: int = DEFAULT_DURATION_MS,
    takeoff_time: float = 0.0,
    takeoff_speed: float = DEFAULT_TAKEOFF_SPEED_M_S,
    landing_speed: float = DEFAULT_LANDING_SPEED_M_S,
    velocity_smoothing: float = DEFAULT_VELOCITY_SMOOTHING,
    ground_positions: Optional[Sequence[Sequence[float]]] = None,
) -> List[dict]:
    """Convert a *SolverResult* into a list of Skybrush trajectory dicts.

    Each dict can be passed directly to ``TrajectorySpecification(d)``
    and has the form::

        {
            "version": 1,
            "takeoffTime": <float>,
            "points": [
                [t, [x, y, z], []],           # linear segment
                [t, [x, y, z], [c1, c2]],     # eased (cubic Bézier) segment
                ...
            ]
        }

    The generated trajectory includes a takeoff segment (ground → first
    waypoint altitude) at the beginning and a landing segment (last
    waypoint altitude → ground) at the end.

    Parameters:
        result: output of ``PathSolver.solve()``
        duration_ms: milliseconds per step (from the API request). Ignored for
            individual steps that already carry an absolute ``time_ms``.
        takeoff_time: seconds to wait on the ground before takeoff
        takeoff_speed: peak vertical speed during takeoff in m/s
        landing_speed: peak vertical speed during landing in m/s
        velocity_smoothing: strength of the speed-ramp smoothing in ``[0, 1]``.
            0 disables it; any value > 0 replaces the linear segments with
            cubic Bézier segments whose control points lie *on* the straight
            line between waypoints, so the path is unchanged but the speed
            ramps up from / down to zero at the trajectory start, end and
            every hold. Larger values additionally slow the drone down at
            direction-change corners (1 = full stop at each corner).
        ground_positions: optional per-drone ``[x, y, z]`` ground positions;
            the z coordinate is used as the ground level for takeoff and
            landing instead of assuming a flat ground at z=0.
    """
    trajectories: List[dict] = []

    for idx, drone in enumerate(result.drones):
        did = drone.drone_id
        points: List[list] = []

        ground_z = 0.0
        if ground_positions is not None and idx < len(ground_positions):
            ground_z = float(ground_positions[idx][2])

        # Collect raw waypoints from solver
        raw_points: List[list] = []
        for rec in result.steps:
            t_sec = round(step_time_sec(rec, duration_ms), 4)
            pos = rec.positions[did]
            raw_points.append(
                [t_sec, [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)], []]
            )

        if not raw_points:
            trajectories.append(
                {"version": 1, "takeoffTime": takeoff_time, "points": []}
            )
            continue

        first_pos = raw_points[0][1]  # [x, y, z]
        last_pos = raw_points[-1][1]

        ground_start = [first_pos[0], first_pos[1], round(ground_z, 4)]
        ground_end = [last_pos[0], last_pos[1], round(ground_z, 4)]

        takeoff_alt, takeoff_duration, _, landing_duration = _takeoff_landing_profile(
            first_pos,
            last_pos,
            ground_z,
            ground_z,
            takeoff_speed,
            landing_speed,
            velocity_smoothing,
        )

        # Build full trajectory:
        # 1) Ground start at t=0
        # 2) Top of takeoff at t=takeoff_duration
        # 3) Solver waypoints shifted by takeoff_duration
        # 4) Landing to ground

        points.append([0, ground_start, []])

        if takeoff_duration > 0:
            points.append([round(takeoff_duration, 4), list(first_pos), []])

        for raw_pt in raw_points:
            t_shifted = round(raw_pt[0] + takeoff_duration, 4)
            # Skip duplicate of first point (already added as takeoff end)
            if takeoff_duration > 0 and raw_pt is raw_points[0]:
                continue
            # At ground level, ground_start at t=0 already matches raw_points[0].
            if takeoff_duration == 0 and raw_pt is raw_points[0] and takeoff_alt <= 0:
                continue
            points.append([t_shifted, list(raw_pt[1]), []])

        last_t = points[-1][0]
        if landing_duration > 0:
            points.append([round(last_t + landing_duration, 4), ground_end, []])

        # Collapse consecutive identical positions into a single constant
        # segment, then ease the speed. Collinear runs are NOT merged for
        # solver output: merging would let the schedule deviation grow with
        # the run length and break the solver's collision guarantees.
        optimised = _collapse_stationary(points)
        smoothed = apply_velocity_smoothing(optimised, velocity_smoothing)

        trajectories.append(
            {
                "version": 1,
                "takeoffTime": takeoff_time,
                "points": smoothed,
            }
        )

    return trajectories


def _normalize_yaw_deg(yaw: float) -> float:
    """Normalize yaw to [-180, 180) for linear setpoint interpolation."""
    yaw = yaw % 360.0
    if yaw >= 180.0:
        yaw -= 360.0
    return yaw


def _lerp_yaw_deg(start: float, end: float, fraction: float) -> float:
    """Linearly interpolate yaw along the shortest path on the circle."""
    delta = _yaw_delta_deg(start, end)
    return _normalize_yaw_deg(start + delta * fraction)


def _yaw_delta_deg(start: float, end: float) -> float:
    """Shortest signed yaw change from *start* to *end* in degrees."""
    return (end - start + 180.0) % 360.0 - 180.0


# Public aliases for other modules in this package (e.g. the extension's
# yaw-transition scheduling) so they don't reach into private helpers.
yaw_delta_deg = _yaw_delta_deg


def lerp_yaw_deg(start: float, end: float, fraction: float) -> float:
    """Public alias of :func:`_lerp_yaw_deg`."""
    return _lerp_yaw_deg(start, end, fraction)


def _append_yaw_setpoint(setpoints: list[list[float]], t: float, yaw: float) -> None:
    key = round(t, 4)
    yaw_rounded = round(_normalize_yaw_deg(yaw), 4)
    if setpoints and setpoints[-1][0] == key:
        setpoints[-1][1] = yaw_rounded
    else:
        setpoints.append([key, yaw_rounded])


def _append_yaw_ramp_setpoints(
    setpoints: list[list[float]],
    *,
    t_start: float,
    t_budget: float,
    yaw_start: float,
    yaw_end: float,
    max_yaw_rate_deg_s: float,
    min_segment_s: float = 0.05,
) -> float:
    """Append yaw setpoints ramping at most *max_yaw_rate_deg_s*.

    Raises :class:`YawRateError` when the change cannot fit *t_budget* at the
    allowed rate — the rate limit is a hard contract, never silently exceeded.
    Returns the ramp end time.
    """
    delta = _yaw_delta_deg(yaw_start, yaw_end)
    if abs(delta) < 1e-9:
        return t_start

    needed = abs(delta) / max_yaw_rate_deg_s
    if t_budget > 0 and needed > t_budget + 1e-6:
        raise YawRateError(
            f"yaw change of {abs(delta):.1f}° needs {needed:.2f}s at "
            f"{max_yaw_rate_deg_s:.0f}°/s but only {t_budget:.2f}s is "
            "available; increase the hold time or lower the yaw change"
        )
    duration = max(needed, 0.001)

    n_segments = max(1, int(math.ceil(duration / min_segment_s)))
    for i in range(1, n_segments + 1):
        frac = i / n_segments
        t = round(t_start + duration * frac, 4)
        yaw = _lerp_yaw_deg(yaw_start, yaw_end, frac)
        _append_yaw_setpoint(setpoints, t, yaw)

    return round(t_start + duration, 4)


def _yaw_at_step(
    record_yaws: Dict[int, float] | None, drone_idx: int, default: float = 0.0
) -> float:
    if not record_yaws:
        return default
    return float(record_yaws.get(drone_idx, default))


def _apply_takeoff_time_to_yaw_setpoints(
    setpoints: list[list[float]], takeoff_time: float
) -> list[list[float]]:
    """Shift yaw setpoints onto the show-wide timeline used by Skybrush."""
    if takeoff_time <= 0:
        return setpoints

    initial_yaw = setpoints[0][1]
    shifted = [[round(t + takeoff_time, 4), yaw] for t, yaw in setpoints]
    if shifted[0][0] > 0:
        shifted.insert(0, [0.0, initial_yaw])
    return shifted


def build_yaw_control_dict(
    result: SolverResult,
    drone_idx: int,
    duration_ms: int,
    *,
    takeoff_time: float = 0.0,
    takeoff_speed: float = DEFAULT_TAKEOFF_SPEED_M_S,
    landing_speed: float = DEFAULT_LANDING_SPEED_M_S,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
    velocity_smoothing: float = DEFAULT_VELOCITY_SMOOTHING,
    ground_positions: Optional[Sequence[Sequence[float]]] = None,
) -> dict[str, Any] | None:
    """Build a Skybrush ``yawControl`` block for one drone.

    Yaw setpoint times follow the same timeline as
    :func:`solver_result_to_trajectory_dicts`, including takeoff and landing
    segments, plus *takeoff_time* (ground wait before the trajectory starts).
    Between setpoints the firmware interpolates yaw linearly.

    In-place yaw changes (same position, different yaw) are spread over time
    according to *max_yaw_rate_deg_s*; changes that cannot fit the available
    time raise :class:`YawRateError`. Runs in O(number of steps).
    """
    if max_yaw_rate_deg_s <= 0:
        raise ValueError("max_yaw_rate_deg_s must be > 0")
    if not result.steps or not any(rec.yaws for rec in result.steps):
        return None

    did = result.drones[drone_idx].drone_id

    records = result.steps  # already ordered by step number
    ground_z = 0.0
    if ground_positions is not None and drone_idx < len(ground_positions):
        ground_z = float(ground_positions[drone_idx][2])

    first_pos = records[0].positions[did]
    last_pos = records[-1].positions[did]
    takeoff_alt, takeoff_duration, _, landing_duration = _takeoff_landing_profile(
        first_pos,
        last_pos,
        ground_z,
        ground_z,
        takeoff_speed,
        landing_speed,
        velocity_smoothing,
    )

    setpoints: list[list[float]] = []

    def yaw_of(rec) -> float:
        return _yaw_at_step(rec.yaws, did)

    _append_yaw_setpoint(setpoints, 0.0, yaw_of(records[0]))
    if takeoff_duration > 0:
        _append_yaw_setpoint(setpoints, takeoff_duration, yaw_of(records[0]))

    prev_rec = None
    for rec in records:
        t_shifted = round(step_time_sec(rec, duration_ms) + takeoff_duration, 4)
        if rec.step == records[0].step:
            if takeoff_duration > 0 or takeoff_alt <= 0:
                prev_rec = rec
                continue
        if prev_rec is not None:
            prev_t_shifted = round(
                step_time_sec(prev_rec, duration_ms) + takeoff_duration, 4
            )
            prev_pos = prev_rec.positions.get(did)
            curr_pos = rec.positions.get(did)
            prev_yaw = yaw_of(prev_rec)
            curr_yaw = yaw_of(rec)
            if (
                prev_pos is not None
                and curr_pos is not None
                and prev_pos == curr_pos
                and abs(_yaw_delta_deg(prev_yaw, curr_yaw)) > 1e-9
            ):
                t_budget = max(t_shifted - prev_t_shifted, 0.001)
                ramp_end = _append_yaw_ramp_setpoints(
                    setpoints,
                    t_start=prev_t_shifted,
                    t_budget=t_budget,
                    yaw_start=prev_yaw,
                    yaw_end=curr_yaw,
                    max_yaw_rate_deg_s=max_yaw_rate_deg_s,
                )
                if ramp_end < t_shifted - 1e-6:
                    _append_yaw_setpoint(setpoints, t_shifted, curr_yaw)
                prev_rec = rec
                continue
        _append_yaw_setpoint(setpoints, t_shifted, yaw_of(rec))
        prev_rec = rec

    last_t = setpoints[-1][0]
    if landing_duration > 0:
        _append_yaw_setpoint(setpoints, last_t + landing_duration, yaw_of(records[-1]))

    if len(setpoints) < 2:
        return None

    setpoints = _apply_takeoff_time_to_yaw_setpoints(setpoints, takeoff_time)

    return {
        "version": 1,
        "autoYaw": False,
        "autoYawOffset": setpoints[0][1],
        "setpoints": setpoints,
    }


def _default_coordinate_system() -> dict:
    """Fallback local NWU coordinate system used when none is supplied."""
    return {"type": "nwu", "origin": [0, 0], "orientation": 0}


def derive_geofence(
    trajectory: dict,
    home: Sequence[float],
    *,
    altitude_margin: float = 10.0,
    distance_margin: float = 20.0,
) -> dict:
    """Derive a per-drone geofence from the trajectory's actual extents.

    The fence hugs the real flight volume plus a margin instead of a fixed
    "wide enough" constant, so a runaway drone is stopped near the show area.
    Control points of eased segments lie on the straight line between the
    keyframes, so keyframe extents already bound the flown path.
    """
    points = trajectory.get("points") or []
    max_alt = 0.0
    max_dist = 0.0
    for _t, pos, _ctrl in points:
        max_alt = max(max_alt, float(pos[2]))
        max_dist = max(max_dist, math.hypot(pos[0] - home[0], pos[1] - home[1]))
    return {
        "version": 1,
        "enabled": True,
        "maxAltitude": math.ceil(max(30.0, max_alt + altitude_margin)),
        "maxDistance": math.ceil(max(50.0, max_dist + distance_margin)),
        "minAltitude": -5.0,
        "action": "land",
        "polygons": [],
        "rallyPoints": [],
    }


def _assemble_show_dict(
    trajectory: dict,
    home: list,
    coordinate_system: dict,
    amsl_reference: Optional[float],
    geofence: Optional[dict] = None,
) -> dict:
    """Wrap a trajectory dict into a full show-specification dict.

    Adds the minimal light program, the geofence (derived from the trajectory
    unless an explicit one is supplied), the home position, the coordinate
    system and (optionally) the AMSL reference — i.e. everything except the
    yaw control block, which is caller-specific.
    """
    import base64

    # Minimal light program: a single END (0x00) byte
    minimal_light = base64.b64encode(b"\x00").decode("ascii")

    show_dict = {
        "trajectory": trajectory,
        "lights": {"version": 1, "data": minimal_light},
        "home": home,
        "coordinateSystem": coordinate_system,
        "geofence": geofence
        if geofence is not None
        else derive_geofence(trajectory, home),
    }
    # Setting an AMSL reference makes the firmware interpret the trajectory Z
    # coordinates as offsets from this absolute altitude (in meters) rather
    # than treating them as relative to home. This corresponds to the "AMSL"
    # altitude reference in the Live UI.
    if amsl_reference is not None:
        show_dict["amslReference"] = float(amsl_reference)

    return show_dict


def build_show_dicts(
    result: SolverResult,
    duration_ms: int = DEFAULT_DURATION_MS,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
    velocity_smoothing: float = DEFAULT_VELOCITY_SMOOTHING,
    takeoff_speed: float = DEFAULT_TAKEOFF_SPEED_M_S,
    landing_speed: float = DEFAULT_LANDING_SPEED_M_S,
    ground_positions: Optional[Sequence[Sequence[float]]] = None,
    geofence: Optional[dict] = None,
) -> List[dict]:
    """Build a list of full *show specification* dicts (one per drone).

    Each dict mirrors what Skybrush Live sends to the server during
    ``OBJ-CMD`` / ``__show_upload``. This is the single builder used for
    saving .skyb files, uploading to UAVs and exporting .skyc, so all three
    always agree byte-for-byte.

    Parameters:
        coordinate_system: optional dict like
            ``{"type": "nwu", "origin": [lon, lat], "orientation": 0}``.
        ground_positions: optional per-drone ground ``[x, y, z]`` used for the
            takeoff/landing profile and as the home position. Defaults to the
            solver's initial positions.
        geofence: optional explicit geofence dict; when omitted a per-drone
            fence is derived from the trajectory extents.
    """
    if coordinate_system is None:
        coordinate_system = _default_coordinate_system()

    traj_dicts = solver_result_to_trajectory_dicts(
        result,
        duration_ms,
        takeoff_time,
        takeoff_speed=takeoff_speed,
        landing_speed=landing_speed,
        velocity_smoothing=velocity_smoothing,
        ground_positions=ground_positions,
    )
    shows: List[dict] = []

    for idx, (drone, traj) in enumerate(zip(result.drones, traj_dicts)):
        if ground_positions is not None and idx < len(ground_positions):
            home_src = ground_positions[idx]
        else:
            home_src = drone.initial
        home = [
            round(float(home_src[0]), 4),
            round(float(home_src[1]), 4),
            round(float(home_src[2]), 4),
        ]

        show_dict = _assemble_show_dict(
            traj, home, coordinate_system, amsl_reference, geofence
        )

        yaw_control = build_yaw_control_dict(
            result,
            idx,
            duration_ms,
            takeoff_time=takeoff_time,
            takeoff_speed=takeoff_speed,
            landing_speed=landing_speed,
            max_yaw_rate_deg_s=max_yaw_rate_deg_s,
            velocity_smoothing=velocity_smoothing,
            ground_positions=ground_positions,
        )
        if yaw_control is not None:
            show_dict["yawControl"] = yaw_control

        shows.append(show_dict)

    return shows


def _delivery_drone_to_trajectory_dict(
    drone: dict,
    takeoff_time: float,
    velocity_smoothing: float,
    *,
    takeoff_speed: float = DEFAULT_TAKEOFF_SPEED_M_S,
    landing_speed: float = DEFAULT_LANDING_SPEED_M_S,
) -> dict:
    """Convert one pre-built delivery drone (``{initial_position, path}``) into a
    Skybrush trajectory dict.

    The delivery payload carries a per-drone waypoint list where each point has
    its own ``durationMs`` (time to travel from the previous point to this one)
    and an optional ``holdMs`` (extra time to hover at the point). Unlike the
    solver output there is no global step timeline, so timing is accumulated
    point-by-point here.

    When the first point is above the drone's ground level (``ground_z`` field
    of the drone entry, default 0), a ground start and a peak-speed-limited
    takeoff segment are prepended, and a matching landing segment is appended,
    so delivery shows behave like generated shows instead of starting mid-air.
    """
    init = drone.get("initial_position") or [0.0, 0.0, 0.0]
    start = [
        round(float(init[0]), 4),
        round(float(init[1]), 4),
        round(float(init[2]), 4),
    ]
    ground_z = float(drone.get("ground_z", 0.0))

    points: List[list] = [[0.0, start, []]]
    t = 0.0
    for raw in drone.get("path", []):
        dur = float(raw.get("durationMs", 0)) / 1000.0
        t = round(t + dur, 3)
        pos = [
            round(float(raw.get("x", 0.0)), 4),
            round(float(raw.get("y", 0.0)), 4),
            round(float(raw.get("z", 0.0)), 4),
        ]
        # Keep time strictly increasing: a zero/negative durationMs would make
        # two keyframes share a timestamp, which the trajectory parser rejects.
        if t <= points[-1][0]:
            t = round(points[-1][0] + 0.001, 3)
        points.append([t, pos, []])

        hold = float(raw.get("holdMs", 0)) / 1000.0
        if hold > 0:
            t = round(t + hold, 3)
            points.append([t, list(pos), []])

    # Wrap with ground start / takeoff / landing when the path flies above
    # the ground level.
    first_pos = points[0][1]
    last_pos = points[-1][1]
    takeoff_alt, takeoff_duration, landing_alt, landing_duration = (
        _takeoff_landing_profile(
            first_pos,
            last_pos,
            ground_z,
            ground_z,
            takeoff_speed,
            landing_speed,
            velocity_smoothing,
        )
    )
    if takeoff_alt > 0:
        points = [[0.0, [first_pos[0], first_pos[1], round(ground_z, 4)], []]] + [
            [round(pt + takeoff_duration, 4), pos, ctrl] for pt, pos, ctrl in points
        ]
    if landing_alt > 0:
        points.append(
            [
                round(points[-1][0] + landing_duration, 4),
                [last_pos[0], last_pos[1], round(ground_z, 4)],
                [],
            ]
        )

    optimised = _collapse_stationary(points)
    # Delivery paths were not produced by the solver, so there is no
    # synchronized schedule to preserve; merging collinear runs is allowed
    # here and the verification gate checks the final result anyway.
    smoothed = apply_velocity_smoothing(
        optimised, velocity_smoothing, merge_collinear=True
    )

    return {"version": 1, "takeoffTime": takeoff_time, "points": smoothed}


def build_delivery_show_dicts(
    drones: List[dict],
    *,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
    velocity_smoothing: float = DEFAULT_VELOCITY_SMOOTHING,
    takeoff_speed: float = DEFAULT_TAKEOFF_SPEED_M_S,
    landing_speed: float = DEFAULT_LANDING_SPEED_M_S,
    geofence: Optional[dict] = None,
) -> List[dict]:
    """Build per-drone show dicts from a pre-built delivery ``drones`` payload.

    Mirrors :func:`build_show_dicts` but takes ready-made per-drone paths (as
    sent by the 3D view's "path delivery") instead of a solver result. No yaw
    control block is emitted (the delivery payload carries no yaw).
    """
    if coordinate_system is None:
        coordinate_system = _default_coordinate_system()

    shows: List[dict] = []
    for drone in drones:
        traj = _delivery_drone_to_trajectory_dict(
            drone,
            takeoff_time,
            velocity_smoothing,
            takeoff_speed=takeoff_speed,
            landing_speed=landing_speed,
        )
        init = drone.get("initial_position") or [0.0, 0.0, 0.0]
        ground_z = float(drone.get("ground_z", 0.0))
        home = [round(float(init[0]), 4), round(float(init[1]), 4), round(ground_z, 4)]
        shows.append(
            _assemble_show_dict(traj, home, coordinate_system, amsl_reference, geofence)
        )

    return shows


async def save_skyb_files(
    show_dicts: List[dict],
    output_dir: str | Path,
) -> Dict[str, str]:
    """Save ready-made per-drone show dicts as ``.skyb`` files + ``show.json``.

    Takes the exact show dicts that are uploaded to the UAVs, so the files on
    disk always match what the drones receive over MAVFTP.

    Returns a dict mapping drone id strings to their ``.skyb`` file paths.
    """
    from flockwave.server.show.formats import SkybrushBinaryShowFile
    from flockwave.server.show.trajectory import TrajectorySpecification
    from flockwave.server.show.yaw_control import encode_yaw_control_from_show

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    skyb_paths: Dict[str, str] = {}

    for idx, show_dict in enumerate(show_dicts):
        drone_id = f"drone-{idx + 1}"

        traj_spec = TrajectorySpecification(show_dict["trajectory"])

        async with SkybrushBinaryShowFile.create_in_memory(version=2) as f:
            await f.add_trajectory(traj_spec)

            # Minimal light program (single END opcode)
            await f.add_encoded_light_program(b"\x00")

            yaw_payload = encode_yaw_control_from_show(show_dict)
            if yaw_payload is not None:
                await f.add_encoded_yaw_setpoints(yaw_payload)

            await f.add_comment(f"path_planner:{drone_id}")
            await f.finalize()
            skyb_data = f.get_contents()

        skyb_path = output_dir / f"{drone_id}.skyb"
        skyb_path.write_bytes(skyb_data)
        skyb_paths[drone_id] = str(skyb_path)

    # ── Combined show JSON ──────────────────────────────────────────
    combined = {
        "version": 1,
        "num_drones": len(show_dicts),
        "drones": {
            f"drone-{idx + 1}": show_dict for idx, show_dict in enumerate(show_dicts)
        },
    }

    show_json_path = output_dir / "show.json"
    show_json_path.write_text(json.dumps(combined, indent=2), encoding="utf-8")
    skyb_paths["_show_json"] = str(show_json_path)

    return skyb_paths


# ---------------------------------------------------------------------------
# Velocity smoothing
# ---------------------------------------------------------------------------


def _merge_collinear_runs(
    points: List[list], angle_threshold_deg: float = CORNER_ANGLE_THRESHOLD_DEG
) -> List[list]:
    """Merge consecutive same-direction (near-collinear) moving segments.

    When A, B, C are travelled in the same direction (within
    ``angle_threshold_deg``), the middle keyframe B is dropped so that A→C
    becomes a single segment spanning the full duration.

    NOTE: with a non-zero angle threshold this is *not* geometry-preserving —
    a dropped keyframe may sit up to ``sin(threshold)`` × segment-length away
    from the merged straight line, and the drone's position at intermediate
    times deviates from the original schedule by up to ~9.7% of the merged
    run's length. It is therefore only used for externally supplied (delivery)
    paths, never for solver output whose collision guarantees depend on the
    schedule. Corners, holds and the trajectory ends always break a run.
    """
    n = len(points)
    if n < 3:
        return points

    def unit(a, b):
        d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        length = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        if length <= 1e-9:
            return None
        return (d[0] / length, d[1] / length, d[2] / length)

    cos_threshold = math.cos(math.radians(angle_threshold_deg))

    result: List[list] = [points[0]]
    for i in range(1, n):
        if len(result) >= 2:
            u1 = unit(result[-2][1], result[-1][1])
            u2 = unit(result[-1][1], points[i][1])
            if u1 is not None and u2 is not None:
                dot = u1[0] * u2[0] + u1[1] * u2[1] + u1[2] * u2[2]
                if dot >= cos_threshold:
                    result[-1] = points[i]
                    continue
        result.append(points[i])

    return result


def _control_distances(
    v0: float, v1: float, length: float, dt: float
) -> tuple[float, float]:
    """Distances of the two Bézier control points along the segment line.

    For a cubic Bézier of duration ``dt`` the endpoint speeds are
    ``3·d1/dt`` and ``3·(L-d2)/dt``, so the control distances encode the
    entry/exit speeds. Clamped so the motion stays monotonic (no overshoot).
    """
    d1 = v0 * dt / 3.0
    d2 = length - v1 * dt / 3.0
    d1 = max(0.0, min(d1, length))
    d2 = max(0.0, min(d2, length))
    if d2 < d1:
        d1 = d2 = 0.5 * (d1 + d2)
    return d1, d2


def _segment_peak_speed(v0: float, v1: float, length: float, dt: float) -> float:
    """Exact peak speed of an eased segment (closed form, no sampling).

    The parametric speed of a collinear cubic Bézier is a quadratic in t,
    so its maximum over [0, 1] is at an endpoint or the interior vertex.
    """
    d1, d2 = _control_distances(v0, v1, length, dt)
    # p'(t) = c + b·t + a·t² (distance per unit parameter)
    c = 3.0 * d1
    b = 6.0 * d2 - 12.0 * d1
    a = 9.0 * d1 - 9.0 * d2 + 3.0 * length
    best = max(c, c + b + a)  # endpoints t=0 and t=1
    if a < -1e-12:  # concave -> interior maximum possible
        tv = -b / (2.0 * a)
        if 0.0 < tv < 1.0:
            best = max(best, c + b * tv + a * tv * tv)
    return best / dt


def apply_velocity_smoothing(
    points: List[list],
    smoothing: float,
    *,
    merge_collinear: bool = False,
    max_velocity_xy: float = MAX_VELOCITY_XY,
    max_velocity_z: float = MAX_VELOCITY_Z,
) -> List[list]:
    """Give the trajectory a smooth speed profile without changing its path.

    The input ``points`` is the list of ``[t, [x, y, z], control]`` keyframes
    (after :func:`_collapse_stationary`) where every segment is linear and
    travelled at constant speed — the speed jumps from 0 to cruise instantly
    at every segment boundary, which is the "inertia" jerk the drone feels.

    Each moving segment is replaced with a **cubic Bézier** whose two interior
    control points lie *on the straight line* between the segment endpoints,
    so the geometric path is unchanged; only the speed along it changes.

    Per-waypoint target speed at the shared keyframe between two segments:

    * trajectory start / end, and any waypoint next to a hold: **0** — the
      drone is genuinely at rest there.
    * direction-change **corner** (angle > ``CORNER_ANGLE_THRESHOLD_DEG``):
      ``(1 - smoothing) · cruise`` (``smoothing == 1`` → full stop).
    * near-collinear pass-through: cruise speed, i.e. the segment stays
      effectively constant-speed (zero schedule deviation).

    Velocity safety: the exact peak speed of every eased segment is checked
    against ``max_velocity_xy`` / ``max_velocity_z``. Segments that would
    exceed a limit get their easing relaxed toward constant speed; if the
    limits still cannot be met the whole trajectory falls back to constant
    speed. If even the *cruise* speed violates a limit, the trajectory is
    unflyable at this timing and :class:`TrajectoryLimitError` is raised.

    ``smoothing`` is clamped to ``[0, 1]``; ``0`` returns the input unchanged.
    ``merge_collinear`` must stay False for solver output (see
    :func:`_merge_collinear_runs`).
    """
    if len(points) < 2:
        return points
    smoothing = min(1.0, max(0.0, smoothing))

    if merge_collinear and smoothing > 0.0:
        points = _merge_collinear_runs(points)
    n = len(points)

    # Per-segment geometry. Segment k (for k in 1..n-1) ends at keyframe k and
    # carries its control points on keyframe k (Skybrush trajectory convention).
    seg_dir: List[Optional[List[float]]] = [None] * n
    seg_len: List[float] = [0.0] * n
    seg_dt: List[float] = [0.0] * n
    seg_cruise: List[float] = [0.0] * n
    for k in range(1, n):
        a = points[k - 1][1]
        b = points[k][1]
        d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
        length = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        dt = points[k][0] - points[k - 1][0]
        seg_len[k] = length
        seg_dt[k] = dt
        if length > 1e-9 and dt > 1e-9:
            seg_dir[k] = [d[0] / length, d[1] / length, d[2] / length]
            seg_cruise[k] = length / dt
        # else: zero-length hold or zero-duration -> constant segment

    # The cruise speed itself must respect the limits — easing can only make
    # peaks higher, never fix an infeasible schedule.
    def axis_limit(direction: List[float]) -> float:
        h_xy = math.hypot(direction[0], direction[1])
        v_z = abs(direction[2])
        limit = math.inf
        if h_xy > 1e-9:
            limit = min(limit, max_velocity_xy / h_xy)
        if v_z > 1e-9:
            limit = min(limit, max_velocity_z / v_z)
        return limit

    seg_limit: List[float] = [math.inf] * n
    for k in range(1, n):
        u = seg_dir[k]
        if u is None:
            continue
        seg_limit[k] = axis_limit(u)
        if seg_cruise[k] > seg_limit[k] * (1.0 + 1e-6):
            raise TrajectoryLimitError(
                f"segment ending at t={points[k][0]:.2f}s requires a cruise "
                f"speed of {seg_cruise[k]:.2f} m/s which exceeds the "
                f"velocity limit of {seg_limit[k]:.2f} m/s; increase the "
                "segment duration"
            )

    if smoothing <= 0.0:
        return points

    # Target speed at each keyframe (see docstring).
    speed_at: List[float] = [0.0] * n
    for i in range(n):
        prev_dir = seg_dir[i] if i >= 1 else None
        next_dir = seg_dir[i + 1] if i + 1 < n else None
        if prev_dir is None or next_dir is None:
            speed_at[i] = 0.0  # start / end / next to a hold -> at rest
            continue
        dot = (
            prev_dir[0] * next_dir[0]
            + prev_dir[1] * next_dir[1]
            + prev_dir[2] * next_dir[2]
        )
        dot = max(-1.0, min(1.0, dot))
        angle_deg = math.degrees(math.acos(dot))
        pass_through = min(seg_cruise[i], seg_cruise[i + 1])
        if angle_deg > CORNER_ANGLE_THRESHOLD_DEG:
            speed_at[i] = (1.0 - smoothing) * pass_through  # corner
        else:
            speed_at[i] = pass_through  # straight-through, keep cruising

    # Peak-speed enforcement: relax the easing (raise endpoint speeds toward
    # cruise) on segments whose eased peak would exceed the velocity limits.
    for _ in range(4):
        any_violation = False
        for k in range(1, n):
            if seg_dir[k] is None:
                continue
            peak = _segment_peak_speed(
                speed_at[k - 1], speed_at[k], seg_len[k], seg_dt[k]
            )
            if peak <= seg_limit[k] * (1.0 + 1e-9):
                continue
            any_violation = True
            cruise = seg_cruise[k]
            # Endpoint speeds and the peak are affine in the ease amount, so
            # the relaxation factor has a closed form.
            beta = (seg_limit[k] - cruise) / max(peak - cruise, 1e-9)
            beta = max(0.0, min(1.0, beta))
            for endpoint in (k - 1, k):
                current = speed_at[endpoint]
                demanded = cruise + beta * (current - cruise)
                bound_candidates = []
                if endpoint >= 1 and seg_dir[endpoint] is not None:
                    bound_candidates.append(seg_cruise[endpoint])
                if endpoint + 1 < n and seg_dir[endpoint + 1] is not None:
                    bound_candidates.append(seg_cruise[endpoint + 1])
                bound = min(bound_candidates) if bound_candidates else cruise
                speed_at[endpoint] = min(max(current, demanded), bound)
        if not any_violation:
            break

    # Final safety check; fall back to constant speed if the limits still
    # cannot be met with easing (constant speed is feasible per the cruise
    # check above).
    for k in range(1, n):
        if seg_dir[k] is None:
            continue
        peak = _segment_peak_speed(speed_at[k - 1], speed_at[k], seg_len[k], seg_dt[k])
        if peak > seg_limit[k] * (1.0 + 1e-6):
            return points

    out: List[list] = [list(p) for p in points]
    for k in range(1, n):
        u = seg_dir[k]
        if u is None:
            out[k][2] = []  # keep holds / degenerate segments constant
            continue
        a = points[k - 1][1]
        d1, d2 = _control_distances(speed_at[k - 1], speed_at[k], seg_len[k], seg_dt[k])
        p1 = [round(a[j] + u[j] * d1, 4) for j in range(3)]
        p2 = [round(a[j] + u[j] * d2, 4) for j in range(3)]
        out[k][2] = [p1, p2]

    return out


def _collapse_stationary(points: List[list]) -> List[list]:
    """Remove consecutive keyframes with identical positions.

    When a drone sits still for multiple steps the algorithm records the
    same (x, y, z) repeatedly.  We collapse those into a single pair of
    keyframes (enter + exit) so the trajectory keeps its timing but the
    encoder produces a compact *constant* segment instead of many tiny
    linear segments with zero displacement.

    The first and last keyframe are always kept.
    """
    if len(points) <= 2:
        return points

    collapsed: List[list] = [points[0]]

    i = 1
    while i < len(points):
        # Look ahead: is the position the same as the previous kept point?
        prev_pos = collapsed[-1][1]
        cur_pos = points[i][1]

        if cur_pos == prev_pos:
            # Skip ahead to the last frame with this same position
            j = i
            while j + 1 < len(points) and points[j + 1][1] == cur_pos:
                j += 1
            # Keep only the exit keyframe (or the final point)
            collapsed.append(points[j])
            i = j + 1
        else:
            collapsed.append(points[i])
            i += 1

    return collapsed
