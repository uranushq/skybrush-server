"""Extension that provides a REST API endpoint for automatic 3D drone
path planning with collision avoidance.

Accepts initial and target positions for multiple drones, runs a greedy
path-planning algorithm, and returns per-drone waypoint paths.
"""

dependencies = ("http_server",)

__all__ = ("construct", "dependencies", "description", "schema")


def __getattr__(name):
    # Lazy so the algorithm modules (solver, converter, verify, ...) stay
    # importable without the server runtime deps (quart, trio) — e.g. from
    # unit tests or offline tooling. The extension manager still finds
    # construct/description/schema through this hook.
    if name in ("construct", "description", "schema"):
        from . import extension

        return getattr(extension, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
