"""Extension that provides a REST API endpoint for automatic 3D drone
path planning with collision avoidance.

Accepts initial and target positions for multiple drones, runs a greedy
path-planning algorithm, and returns per-drone waypoint paths.
"""

from .extension import construct, description, schema

dependencies = ("http_server",)

__all__ = ("construct", "dependencies", "description", "schema")
