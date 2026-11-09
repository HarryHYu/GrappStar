"""End-to-end test: gs_client against mock_gs, then mission.py against mock_dart + mock_gs. Prints ALL PASS."""
import time, mock_dart, mock_gs
from gs_client import GroundStation, Refused
from mission import Mission, dry_run_setup

LOG = []
def log(s): LOG.append(s)
def reset_dart(): mock_dart.S.update(seq="idle", event="boot", keys=[], t0=None, armed=False, landed=False)

sd, sg, d, g = dry_run_setup()

# client basics
s = g.status(); assert s["state"] == "ready" and s["switches"]["seated"]; print("ok: status READY, seated")
g.cock(); g.fire()   # cock when READY = "already cocked", then fire
try: g.retrieve(); raise AssertionError("retrieve accepted while firing")
except Refused as e: assert "busy" in str(e); print("ok: command refused while firing (409 -> Refused)")
g.wait_state("payout", limit=5); g.stop(); mock_gs.S.update(seated=True, cocked=True, line_m=0); g.stop()
try: g.limit(500); raise AssertionError("limit 500 accepted")
except Refused: print("ok: limit outside 2..50 N refused")
g.cock(); s = g.wait_state("ready", limit=5); print("ok: cock -> READY")
try: g.lower(); raise AssertionError("lower accepted with the rail down")
except Refused as e: assert "already down" in str(e); print("ok: lower refused with the rail down")
mock_gs.S.update(state="idle", phi=270.0, phi_known=False, low=False, cocked=False, seated=False)   # power cut mid-fire: rail up, phi unknown
for f in (g.cock, g.finish):
    try: f(); raise AssertionError("B accepted with the rail up and phi unknown")
    except Refused as e: assert "lower" in str(e)
g.lower(); s = g.wait_state("idle", "ready", limit=5); assert s["phi_known"] and s["switches"]["low"] and s["phi"] == 333.0
print("ok: rail up, phi unknown: cock and finish point to lower; lower -> rail down, phi 333")
mock_gs.S.update(seated=True, cocked=True, line_m=0, shot=False); g.cock(); g.wait_state("ready", limit=5)
mock_gs.S["state"] = "idle"; mock_gs.S["keys"].clear(); g.hold("f", 1.0)
keys = [k for k in mock_gs.S["keys"] if k[1] == "f"]; gaps = [b[0] - a[0] for a, b in zip(keys, keys[1:])]
assert 5 <= len(keys) <= 9 and max(gaps) < 0.35 and mock_gs.S["cmds"][-1][1] == "stop" and mock_gs.S["state"] != "jog", (len(keys), gaps)
print(f"ok: jog hold sends {len(keys)} repeats, max gap {max(gaps) * 1000:.0f} ms (< 350 ms deadman), then stop")

# review rules mirrored in the mock: finish only on request, F refused part-drawn unless force
mock_gs.S.clear(); mock_gs.S.update(mock_gs.fresh()); mock_gs.S.update(state="idle", phi=100.0, cocked=False, seated=False, line_m=3.0)
try: g.tow(); raise AssertionError("tow accepted part-drawn")
except Refused as e: assert "partly drawn" in str(e)
g.cmd("force"); g.tow(); g.stop(); print("ok: F refused with the carriage partly drawn, force allows one")
mock_gs.S.clear(); mock_gs.S.update(mock_gs.fresh()); mock_gs.S.update(seated=False)
g.cmd("finish"); s = g.wait_state("idle", limit=5); assert s["phi"] == 333.0 and "finished" in s["event"]; print("ok: finish runs until the rail is down")

# mission, ground station driven step by step
mock_gs.S.clear(); mock_gs.S.update(mock_gs.fresh()); reset_dart()
m = Mission(d, g, log=log, release_s=(0.5, 0.5), land_timeout=10, anchor_timeout=10)
t0 = time.time(); s = m.run(); dt = time.time() - t0
assert m.steps == ["preflight", "fire", "wait landing", "anchor", "tow", "recover", "release dart", "cock", "retrieve", "done"], m.steps
assert s["state"] == "ready" and s["switches"]["seated"] and mock_dart.S["seq"] == "anchored"
dk = [k[1] for k in mock_dart.S["keys"]]
assert "A" in dk and "#" in dk and "!" in dk and "$" not in dk and "4" not in dk and dk[-1] == " "   # v34: release = drills out + pods up, no foot
cm = [c[1] for c in mock_gs.S["cmds"]]
assert cm.index("fire") < cm.index("tow") < cm.index("cock") < cm.index("retrieve"), cm
print(f"ok: mission (step by step): fire, land, anchor, tow to hold, release, cock, retrieve -> READY in {dt:.1f} s")

# mission using the ground station's cycle helper
mock_gs.S.clear(); mock_gs.S.update(mock_gs.fresh()); reset_dart()
m = Mission(d, g, log=log, release_s=(0.5, 0.5), land_timeout=10, anchor_timeout=10)
s = m.run_cycle()
assert s["state"] == "ready" and s["switches"]["seated"] and s["cycle"] == "off" and "cycle complete" in s["event"], s
assert any(c[1] == "anchored" for c in mock_gs.S["cmds"]) and any(c[1] == "released" for c in mock_gs.S["cmds"])
print("ok: mission (cycle helper): anchored / released flags drive the ground station to READY")

# fault during tow: mission aborts and stops both
mock_gs.S.clear(); mock_gs.S.update(mock_gs.fresh()); reset_dart()
m = Mission(d, g, log=log, release_s=(0.5, 0.5), land_timeout=10, anchor_timeout=10)
m.recover_fn = lambda _: None
orig_tow = g.tow
def tow_then_fault(dist=0): r = orig_tow(dist); mock_gs.S["inject_fault"] = "over current"; return r
g.tow = tow_then_fault
try: m.run(); raise AssertionError("fault not raised")
except RuntimeError as e: assert "FAULT" in str(e), e
g.tow = orig_tow
assert "mission aborted: stopping both" in LOG and mock_dart.S["keys"][-1][1] == " " and mock_gs.S["state"] == "fault"
print("ok: ground station FAULT mid-tow -> mission aborts, both stopped")

# not READY: preflight refuses to fire
g.clear(); mock_gs.S.update(cocked=False); g.stop(); reset_dart()
try: Mission(d, g, log=log).run(); raise AssertionError("fired while not READY")
except RuntimeError as e: assert "not READY" in str(e); print("ok: preflight refuses when the ground station is not READY")

sd.shutdown(); sg.shutdown(); print("ALL PASS")
