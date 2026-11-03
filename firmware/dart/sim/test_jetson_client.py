#!/usr/bin/env python3
"""Contract C2 end to end: the Jetson's own dart client (jetson/grappstar/comms/dart.py) against the REAL dart sketch
running in the host simulator (./kedge_sim). Standard library only (the client module is loaded by file path, so the
rest of the jetson package and its dependencies are not needed).

    make && python3 test_jetson_client.py

The tests share one simulator and run in order. A mission as the Jetson runs it (log-only arm, launch, landing,
flight log, /auto, release) plus the refusals the client must handle.
"""
import importlib.util
import json
import os
import socket
import subprocess
import sys
import time
import unittest
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "kedge_sim")
LOG = os.path.join(HERE, "build", "test_jetson_client.log")
CLIENT = os.path.normpath(os.path.join(HERE, "..", "..", "..", "jetson", "grappstar", "comms", "dart.py"))


def load_client():
    spec = importlib.util.spec_from_file_location("jetson_dart_client", CLIENT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(os.path.exists(CLIENT), "jetson client not in this checkout")
class JetsonClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not os.path.exists(SIM):
            subprocess.check_call(["make", "-C", HERE])
        os.makedirs(os.path.dirname(LOG), exist_ok=True)
        cls.port = free_port()
        cls.log = open(LOG, "w")
        cls.proc = subprocess.Popen([SIM, "--port", str(cls.port)], stdin=subprocess.DEVNULL, stdout=cls.log, stderr=subprocess.STDOUT)
        cls.m = load_client()
        cls.d = cls.m.Dart(f"127.0.0.1:{cls.port}", timeout=2.0)
        t0 = time.time()
        while True:
            try:
                cls.d.status()
                break
            except cls.m.DartError:
                if cls.proc.poll() is not None or time.time() - t0 > 10:
                    raise RuntimeError("simulator did not start, see " + LOG)
                time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(5)
        cls.log.close()

    def sim(self, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=3) as r:
            body = r.read().decode()
        try:
            return json.loads(body or "{}")
        except ValueError:
            return {"text": body}

    def test_01_status_phase_clock(self):
        s = self.d.status()
        self.assertEqual(s["phase"], "IDLE")
        self.assertEqual(s["fw"], "kedge-dart-v4")
        self.assertEqual(s["warnings"], [])
        self.assertIsNotNone(self.d.clock_offset())
        self.assertTrue(self.d.link()["connected"])

    def test_02_arm_log_only_and_refusals_while_armed(self):
        s = self.d.arm(True, auto=False)
        self.assertTrue(s["armed"]); self.assertFalse(s["auto_on_landing"]); self.assertEqual(s["phase"], "ARMED")
        with self.assertRaises(self.m.DartRefused) as cm:
            self.d.flightlog(timeout=2.0, tries=1)
        self.assertTrue(cm.exception.reason.startswith("busy"), cm.exception.reason)
        self.assertTrue(self.d.link()["connected"], "a 409 must not count as a link failure")

    def test_02b_auto_while_armed_disarms_first(self):
        # the launch was never seen: the Jetson's /auto disarms and starts (no deadlock), then we stop and re-arm
        self.d.auto()
        s = self.d.status()
        self.assertFalse(s["armed"]); self.assertEqual(s["phase"], "ANCHORING")
        self.d.stop()
        time.sleep(0.3)
        try:
            self.d.hold("@", 1.0)                # bring the kick back (ends with a "stalled" refusal at the stop)
        except self.m.DartRefused as e:
            self.assertTrue(e.reason.startswith("stalled"), e.reason)
        self.assertEqual(self.sim("/sim/state")["kick_pos"], 0)
        s = self.d.arm(True, auto=False)
        self.assertTrue(s["armed"])

    def test_03_launch_landing_flightlog(self):
        time.sleep(2.5)                          # the dart sits armed on the launcher for a while
        self.sim("/sim/launch")
        s = self.d.wait_phase(("LANDED",), 10.0)
        self.assertTrue(s["landed"]); self.assertIsInstance(s["t_launch_ms"], int); self.assertIsInstance(s["t_land_ms"], int)
        t0 = time.time()
        while not self.d.status()["log_frozen"] and time.time() - t0 < 6:
            time.sleep(0.2)
        fl = self.d.flightlog()
        self.assertEqual(fl["hz"], 50); self.assertTrue(fl["frozen"]); self.assertEqual(fl["landing"], "impact")
        self.assertEqual(fl["n"], len(fl["samples"]))
        cols = self.m.flightlog_arrays(fl)
        self.assertGreater(len(cols["t"]), 150)
        self.assertLessEqual(cols["t"][0] * 1000, fl["t_launch_ms"] - 1900)     # 2 s before launch
        self.assertGreaterEqual(cols["t"][-1] * 1000, fl["t_land_ms"] + 2900)   # 3 s after landing
        self.assertGreater(max(cols["amag"]), 4 * 9.81)                       # the launch push is in the log
        off = self.d.clock_offset()
        self.assertLess(abs(off + fl["t_launch_ms"] / 1000.0 - time.time()), 30)  # dart clock maps near now

    def test_04_auto_anchors(self):
        self.d.auto()
        s = self.d.wait_phase("ANCHORED", 45.0)
        self.assertEqual(s["seq"], "anchored"); self.assertEqual(s["warnings"], [])
        st = self.sim("/sim/state")
        self.assertEqual(st["kick_pos"], 4300); self.assertEqual(st["deploy_pos"], 1400)
        with self.assertRaises(self.m.DartRefused) as cm:
            self.d.auto()
        self.assertIn("already anchored", cm.exception.reason)

    def test_05_release_completes_stowed(self):
        self.d.release()
        self.d.release()                         # a repeat while releasing is accepted (no restart)
        s = self.d.wait_phase(("IDLE",), 60.0, want_seq="released")
        self.assertEqual(s["seq"], "released")
        st = self.sim("/sim/state")
        self.assertEqual(st["kick_pos"], 0); self.assertEqual(st["deploy_pos"], 0)

    def test_06_hold_and_stop(self):
        self.d.hold("2", 1.0)                    # kick out for 1 s with deadman repeats, then /stop
        st = self.sim("/sim/state")
        self.assertGreater(st["kick_pos"], 500)
        self.assertEqual(self.d.status()["motors"]["kick"], 0)
        try:
            self.d.hold("@", 2.0)                # back again: stalls at stowed, the firmware stops it and refuses
        except self.m.DartRefused as e:          # the next repeat ("stalled: release the key first"); hold() raises
            self.assertTrue(e.reason.startswith("stalled"), e.reason)
        self.assertEqual(self.sim("/sim/state")["kick_pos"], 0)
        self.assertEqual(self.d.status()["motors"]["kick"], 0)

    def test_07_stop_keep_arm(self):
        self.d.arm(True, auto=True)
        self.d.stop(disarm=False)
        self.assertTrue(self.d.status()["armed"])
        self.d.stop()
        self.assertFalse(self.d.status()["armed"])

    def test_08_wait_phase_fails_fast_on_fault(self):
        self.sim("/sim/set?drill_jam=1")
        try:
            # anchor with drills jamming forward is fine (stalls count as done); release with drills jammed faults
            self.d.auto()
            self.d.wait_phase("ANCHORED", 45.0)
            self.d.release()
            with self.assertRaises(self.m.DartError) as cm:
                self.d.wait_phase(("IDLE",), 30.0, want_seq="released")
            self.assertIn("FAULT", str(cm.exception))
        finally:
            self.sim("/sim/set?drill_jam=0")
            self.d.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
