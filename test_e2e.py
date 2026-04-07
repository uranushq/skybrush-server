"""Self-contained test: starts server as subprocess, calls API, prints result."""
import json
import subprocess
import sys
import time
import urllib.request

SERVER_DIR = r"C:\Users\bjw02\OneDrive\Desktop\DCS\skybrush-server"
URL = "http://localhost:5000/api/v1/path-planner/plan"

print("Starting server...")
server = subprocess.Popen(
    [sys.executable, "-m", "uv", "run", "skybrushd"],
    cwd=SERVER_DIR,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
)

# Wait for server to be ready
for i in range(20):
    time.sleep(1)
    try:
        urllib.request.urlopen("http://localhost:5000/", timeout=2)
    except urllib.error.HTTPError:
        print(f"  Server ready! ({i+1}s)")
        break
    except Exception:
        print(f"  Waiting... ({i+1}s)")
else:
    print("Server did not start in time")
    server.kill()
    sys.exit(1)

# Send test request
payload = {
    "initial": [[-5, 0, 0], [0, 0, 0], [5, 0, 0]],
    "target":  [[5, 5, 10], [-5, -5, 10], [0, 10, 5]],
    "step_size": 1.0,
    "duration_ms": 300,
    "seed": 42,
}

data = json.dumps(payload).encode()
req = urllib.request.Request(URL, data=data, headers={"Content-Type": "application/json"})

try:
    with urllib.request.urlopen(req, timeout=60) as resp:
        result = json.loads(resp.read())
        print(f"\n=== STATUS: {resp.status} ===")
        print(f"success: {result.get('success')}")
        print(f"total_steps: {result.get('total_steps')}")
        print(f"drones: {len(result.get('drones', []))}")
        files = result.get("skybrush_files", {})
        if files:
            print(f"\nSaved files:")
            for k, v in files.items():
                print(f"  {k}: {v}")
        if "skybrush_files_error" in result:
            print(f"\nFILE SAVE ERROR: {result['skybrush_files_error']}")
except urllib.error.HTTPError as e:
    print(f"HTTP {e.code}: {e.read().decode()}")
except Exception as e:
    print(f"ERROR: {e}")
finally:
    print("\nShutting down server...")
    server.terminate()
    server.wait(timeout=10)
    print("Done.")
