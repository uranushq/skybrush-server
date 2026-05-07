"""Quick test script for the path-planner API endpoint."""
import json
import urllib.request

url = "http://localhost:5000/api/v1/path-planner/plan"
payload = {
    "initial": [[-5, 0, 0], [0, 0, 0], [5, 0, 0]],
    "target":  [[5, 5, 10], [-5, -5, 10], [0, 10, 5]],
    "step_size": 1.0,
    "duration_ms": 300,
    "seed": 42,
}

data = json.dumps(payload).encode("utf-8")
req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")

try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        result = json.loads(resp.read().decode("utf-8"))
        print("=== STATUS:", resp.status, "===")
        print(json.dumps(result, indent=2))
except urllib.error.URLError as e:
    print("ERROR:", e)
except Exception as e:
    print("ERROR:", type(e).__name__, e)
