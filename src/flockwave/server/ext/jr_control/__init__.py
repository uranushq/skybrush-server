"""JR-board control extension.

Provides a REST API to (a) broadcast the GNSS-PPS ARM sync packet over UDP and
(b) proxy the JR LED boards' on-board HTTP API (health / reboot / redownload).
"""

from .extension import construct, description, schema

dependencies = ("http_server",)

__all__ = ("construct", "dependencies", "description", "schema")
