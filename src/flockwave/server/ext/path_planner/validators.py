"""Pluggable validation framework for the path-planner extension.

The validators here run *before* the solver kicks in. Each validator can
declare a list of MAVLink/firmware parameter names it needs from the
connected drone(s). The extension fetches those parameters once, and then
hands them to every validator together with the request payload via a
:class:`ValidationContext`.

Adding a new check is therefore a 3-step process:

1. Subclass :class:`PathValidator`, declare ``required_params`` (optional)
   and implement :meth:`PathValidator.validate`.
2. Yield :class:`ValidationIssue` objects for any problems.
3. Add an instance of the new validator to :data:`VALIDATORS` (or call
   :func:`register_validator`).

The dispatcher will short-circuit the request with HTTP 422 if any
``error``-level issue is produced. ``warning``-level issues do not block
the request but are surfaced in the response payload.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, List, Mapping, Optional, Sequence, Tuple, cast

__all__ = (
    "ValidationIssue",
    "ValidationContext",
    "PathValidator",
    "MinFlightAltitudeValidator",
    "VALIDATORS",
    "register_validator",
    "fetch_required_params",
    "run_validators",
)


# ---------------------------------------------------------------------------
# Core data types
# ---------------------------------------------------------------------------


SEVERITY_ERROR = "error"
SEVERITY_WARNING = "warning"


@dataclass
class ValidationIssue:
    """A single problem produced by a validator."""

    code: str
    """Stable machine-readable identifier (e.g. ``MIN_FLIGHT_ALTITUDE``)."""

    message: str
    """Human-readable description (English; clients may localize via code)."""

    severity: str = SEVERITY_ERROR
    """``error`` blocks the request, ``warning`` does not."""

    details: dict = field(default_factory=dict)
    """Free-form structured data (offending indices, thresholds, ...)."""

    validator: str = ""
    """Name of the validator that produced the issue (filled in by dispatcher)."""

    def to_dict(self) -> dict:
        return {
            "code": self.code,
            "severity": self.severity,
            "message": self.message,
            "details": self.details,
            "validator": self.validator,
        }


@dataclass
class ValidationContext:
    """Everything a validator needs to do its job.

    Attributes:
        initial:  list of ``[x, y, z]`` start coordinates from the request
        target:   list of ``[x, y, z]`` end coordinates from the request
        step_size: ``step_size`` field from the request (>0)
        duration_ms: ``duration_ms`` field from the request (>0)
        takeoff_time: ``takeoff_time`` field from the request (≥0)
        uav_params: parameter snapshot read from the *first* connected
            drone, keyed by parameter name. Missing parameters are absent
            from the dict (they could not be read or do not exist).
        body: the original request body for validators that need to look
            at additional fields without forcing every new field through
            this dataclass.
    """

    initial: Sequence[Sequence[float]]
    target: Sequence[Sequence[float]]
    step_size: float
    duration_ms: int
    takeoff_time: float
    uav_params: Mapping[str, float]
    body: Mapping[str, Any] = field(default_factory=dict)


class PathValidator:
    """Base class for path validators."""

    name: str = "validator"
    """Stable identifier, surfaced in the API response."""

    required_params: Tuple[str, ...] = ()
    """Firmware parameter names this validator wants pre-fetched."""

    def validate(self, ctx: ValidationContext) -> Iterable[ValidationIssue]:
        """Yield zero or more :class:`ValidationIssue` for the given context."""
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Concrete validators
# ---------------------------------------------------------------------------


class MinFlightAltitudeValidator(PathValidator):
    """Reject paths whose waypoints sit below the firmware's minimum show
    altitude.

    The Skybrush ArduPilot fork rejects ``RELOAD_SHOW`` if the trajectory
    has any keyframe lower than the configured minimum show altitude. The
    parameter name varies between firmware revisions, so we accept a list
    and use the first one we can read.
    """

    name = "min_flight_altitude"

    def __init__(
        self,
        param_names: Sequence[str] = (
            "SHOW_TAKEOFF_ALT_M",
            "SHOW_MIN_ALT_M",
        ),
        fallback_min_alt: float = 2.5,
    ) -> None:
        self.param_names = tuple(param_names)
        self.fallback_min_alt = float(fallback_min_alt)

    @property
    def required_params(self) -> Tuple[str, ...]:  # type: ignore[override]
        return self.param_names

    def _resolve_min_alt(
        self, params: Mapping[str, float]
    ) -> Tuple[float, Optional[str]]:
        for name in self.param_names:
            if name in params:
                try:
                    return float(params[name]), name
                except (TypeError, ValueError):
                    continue
        return self.fallback_min_alt, None

    def validate(self, ctx: ValidationContext) -> Iterable[ValidationIssue]:
        min_alt, source = self._resolve_min_alt(ctx.uav_params)
        # A small numeric tolerance avoids false positives from FP noise
        # (e.g. user typed 2.5 against a 2.5 m limit).
        EPS = 1e-3

        violations: List[dict] = []
        body = ctx.body if isinstance(ctx.body, Mapping) else {}
        drones = body.get("drones")
        if (
            isinstance(drones, list)
            and drones
            and all(isinstance(d, dict) and "path" in d for d in drones)
        ):
            for di, d in enumerate(drones):
                d_map = cast(dict[str, Any], d)
                path = d_map.get("path")
                if not isinstance(path, list):
                    continue
                flight_started = False
                for wi, wp in enumerate(path):
                    if not isinstance(wp, dict) or "z" not in wp:
                        continue
                    wp_map = cast(dict[str, Any], wp)
                    try:
                        z = float(wp_map["z"])
                    except (TypeError, ValueError):
                        continue
                    # Explicit waypoint mode often starts on the ground.
                    # Ignore pre-flight keyframes until the first point
                    # reaches the minimum altitude.
                    if not flight_started and z + EPS < min_alt:
                        continue
                    flight_started = True
                    if z + EPS < min_alt:
                        violations.append(
                            {
                                "label": f"drones[{di}].path",
                                "index": wi,
                                "z": z,
                            }
                        )
        else:
            for label, points in (("initial", ctx.initial), ("target", ctx.target)):
                for i, pt in enumerate(points):
                    if len(pt) < 3:
                        continue
                    z = float(pt[2])
                    if z + EPS < min_alt:
                        violations.append({"label": label, "index": i, "z": z})

        if not violations:
            return

        if source is None:
            source_msg = f"firmware parameter not readable, using fallback {min_alt} m"
        else:
            source_msg = f"firmware parameter {source} = {min_alt} m"

        yield ValidationIssue(
            code="MIN_FLIGHT_ALTITUDE",
            severity=SEVERITY_ERROR,
            message=(
                f"Path 컴파일 오류: invalid height — "
                f"{len(violations)} waypoint(s) below the minimum show "
                f"altitude ({min_alt} m, {source_msg}). Increase the z "
                f"coordinate of the listed waypoints."
            ),
            details={
                "min_alt_m": min_alt,
                "param_source": source,
                "violations": violations,
            },
        )


# ---------------------------------------------------------------------------
# Registry and dispatcher
# ---------------------------------------------------------------------------


VALIDATORS: List[PathValidator] = [
    MinFlightAltitudeValidator(),
]
"""Default ordered list of validators run on every plan request."""


def register_validator(validator: PathValidator) -> PathValidator:
    """Append a validator to the global registry. Returns the validator
    so this can be used as a decorator on a class instance.
    """
    VALIDATORS.append(validator)
    return validator


def collect_required_params(
    validators: Sequence[PathValidator] | None = None,
) -> Tuple[str, ...]:
    """Return the de-duplicated set of parameter names every validator wants."""
    seen: dict[str, None] = {}
    for v in validators if validators is not None else VALIDATORS:
        for p in v.required_params:
            seen.setdefault(p, None)
    return tuple(seen.keys())


async def fetch_required_params(
    uav: Any,
    names: Sequence[str],
    *,
    log: Any = None,
) -> dict[str, float]:
    """Read the given parameters from a UAV. Missing/unreadable params are
    silently dropped from the returned dict — the validator decides how to
    handle their absence.
    """
    out: dict[str, float] = {}
    if uav is None or not names:
        return out

    get_param = getattr(uav, "get_parameter", None)
    if get_param is None:
        if log:
            log.warning(
                "UAV does not expose get_parameter(); skipping parameter validation."
            )
        return out

    for name in names:
        try:
            value = await get_param(name)
        except Exception as exc:
            if log:
                log.debug(f"Could not read param {name}: {exc}")
            continue
        try:
            out[name] = float(value)
        except (TypeError, ValueError):
            if log:
                log.debug(f"Param {name} is not a float: {value!r}")
    return out


def run_validators(
    ctx: ValidationContext,
    validators: Sequence[PathValidator] | None = None,
) -> List[ValidationIssue]:
    """Run every validator and collect their issues, tagged with the
    producing validator's name.
    """
    issues: List[ValidationIssue] = []
    for v in validators if validators is not None else VALIDATORS:
        try:
            for issue in v.validate(ctx) or ():
                issue.validator = v.name
                issues.append(issue)
        except Exception as exc:  # never let a buggy validator break the API
            issues.append(
                ValidationIssue(
                    code="VALIDATOR_INTERNAL_ERROR",
                    severity=SEVERITY_WARNING,
                    message=f"Validator {v.name!r} raised: {exc}",
                    validator=v.name,
                )
            )
    return issues
