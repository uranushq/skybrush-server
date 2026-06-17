"""LED-generator extension.

Compiles a per-pixel "LED show" authored in Skybrush Live into per-drone
``.bin`` files (the format consumed by the JR LED boards) and uploads them to
the external download server.
"""

from .extension import construct, description, schema

dependencies = ("http_server",)

__all__ = ("construct", "dependencies", "description", "schema")
