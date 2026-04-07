"""Quick test: call /api/v1/path-planner/plan and verify auto-upload."""

import json
import urllib.request

url = "http://localhost:5000/api/v1/path-planner/plan"
payload = {
    "initial": [[0, 0, 5], [10, 0, 5], [20, 0, 5]],
    "target":  [[20, 0, 5], [0, 0, 5], [10, 0, 5]],
    "step_size": 1.0,
    "duration_ms": 300,
    "seed": 42,
    "auto_upload": True,
    "coordinate_system": {
        "type": "nwu",
        "origin": [18.915125, 47.486305],
        "orientation": 0,
    },
}

data = json.dumps(payload).encode()
req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})

try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        body = json.loads(resp.read())
        print("=== Response ===")
        print(f"success:     {body.get('success')}")
        print(f"total_steps: {body.get('total_steps')}")
        print(f"num drones:  {len(body.get('drones', []))}")
        print()

        upload = body.get("upload", {})
        print("=== Upload ===")
        print(f"uploaded:   {upload.get('uploaded')}")
        print(f"total_uavs: {upload.get('total_uavs')}")
        print(f"details:    {json.dumps(upload.get('details', {}), indent=2)}")

        if upload.get("error"):
            print(f"error: {upload['error']}")

        skyb = body.get("skybrush_files", {})
        print()
        print("=== Skybrush files ===")
        for k, v in skyb.items():
            print(f"  {k}: {v}")

except Exception as e:
    print(f"ERROR: {e}")
