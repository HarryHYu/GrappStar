import time, mock_dart
from kedge_client import Dart
srv = mock_dart.serve(); d = Dart("127.0.0.1:8780")
s = d.status(); assert s["seq"] == "idle"; print("ok: status")
t = time.time(); d.hold("2", 1.0); keys = [k for k in mock_dart.S["keys"] if k[1] == "2"]
gaps = [b[0] - a[0] for a, b in zip(keys, keys[1:])]
assert 5 <= len(keys) <= 9 and max(gaps) < 0.35 and mock_dart.S["keys"][-1][1] == " ", (len(keys), gaps)
print(f"ok: hold sends {len(keys)} repeats, max gap {max(gaps)*1000:.0f} ms (< 350 ms deadman), then stop")
d.auto(); s = d.wait_auto(poll=0.1, limit=10); assert s["seq"] == "anchored"; print("ok: auto followed to anchored")
srv.shutdown(); print("ALL PASS")
