"""Quick API test — sends POST to the path-planner endpoint."""
import json, urllib.request

url = "http://localhost:5000/api/v1/path-planner/plan"
payload = {
    "initial": [[-5, 0, 0], [0, 0, 0], [5, 0, 0]],
    "target":  [[5, 5, 10], [-5, -5, 10], [0, 10, 5]],
    "step_size": 1.0,
    "duration_ms": 300,
    "seed": 42,
}

data = json.dumps(payload).encode()
req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})

try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        result = json.loads(resp.read())
        print(f"STATUS: {resp.status}")
        print(f"success: {result.get('success')}")
        print(f"total_steps: {result.get('total_steps')}")
        print(f"drones: {len(result.get('drones', []))}")
        files = result.get("skybrush_files", {})
        print(f"\nSaved files:")
        for k, v in files.items():
            print(f"  {k}: {v}")
        if "skybrush_files_error" in result:
            print(f"\nERROR: {result['skybrush_files_error']}")
except urllib.error.HTTPError as e:
    print(f"HTTP {e.code}: {e.read().decode()}")
except Exception as e:
    print(f"ERROR: {e}")
