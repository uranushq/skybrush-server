"""Upload compiled ``.bin`` files to the external download server.

The download server (see ``JR_precise_timing`` firmware ``CFG_DOWNLOAD_SERVER``)
accepts a multipart ``POST /upload`` and responds with JSON::

    {
      "filename": "file12.bin",
      "message":  "File uploaded successfully",
      "url":      "/download/12"
    }

JR boards later fetch their file from ``GET /download/<client_id>``.
"""

from __future__ import annotations

from typing import Any

import httpx

__all__ = ("DEFAULT_UPLOAD_URL", "UploadError", "upload_bin")

DEFAULT_UPLOAD_URL = "http://14.57.192.249:5500/upload"


class UploadError(RuntimeError):
    """Raised when an upload to the download server fails."""


async def upload_bin(
    filename: str,
    data: bytes,
    *,
    url: str = DEFAULT_UPLOAD_URL,
    timeout: float = 30.0,
) -> dict[str, Any]:
    """POST a single ``.bin`` file to the download server.

    Returns the parsed JSON response (expected to contain ``filename``,
    ``message`` and ``url``).

    Raises:
        UploadError: on transport errors, non-2xx responses or invalid JSON.
    """
    files = {"file": (filename, data, "application/octet-stream")}
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.post(url, files=files)
    except httpx.HTTPError as exc:
        raise UploadError(f"could not reach upload server at {url}: {exc}") from exc

    if response.status_code // 100 != 2:
        raise UploadError(
            f"upload server returned HTTP {response.status_code}: "
            f"{response.text[:200]}"
        )

    try:
        return response.json()
    except ValueError as exc:
        raise UploadError(
            f"upload server returned non-JSON response: {response.text[:200]}"
        ) from exc
