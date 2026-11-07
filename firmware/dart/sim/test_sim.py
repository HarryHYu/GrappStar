#!/usr/bin/env python3
"""End to end test of the KEDGE dart HTTP API against the host simulator (standard library only).

Starts ./kedge_sim (the real kedge_dart.ino built against the stubs, see Makefile) on a free port, then drives it over
HTTP the way the Jetson and the web page do. The simulator runs in real time, so the whole run takes about a minute.
The tests share one simulator and run in order (test_01 .. test_11): later ones start from the state earlier ones leave.

    make && python3 test_sim.py          (or: make test)
Simulator output (the sketch's Serial prints) is saved to build/test_sim.log. Extra simulator options can be passed in
the SIM_ARGS environment variable, e.g. SIM_ARGS="--millis-start 4294950000" to cross the millis() wrap mid-run.
"""
import http.client
import json
import os
import shlex
import socket
import statistics
import subprocess
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "kedge_sim")
LOG = os.path.join(HERE, "build", "test_sim.log")
JETSON_CLIENT_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "..", "kedge", "fw", "jetson"))

STATUS_FIELDS = ["fw", "t_ms", "phase", "fault", "rail_V", "current_mA", "peak_mA", "speed", "motors", "seq", "armed",
                 "auto_on_landing", "landed", "roll", "pitch", "accel_g", "ina_ok", "imu_ok", "t_launch_ms", "t_land_ms",
                 "log_frozen", "reset", "event", "video", "batt", "batt_V", "orient", "tilt_deg"]
MOTORS = ["deploy", "kick", "drills", "foot"]


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class SimTest(unittest.TestCase):
    port = None
    proc = None

    # ------------------------------------------------------------------------------------------------ simulator
    @classmethod
    def setUpClass(cls):
        if not os.path.exists(SIM):
            subprocess.check_call(["make", "-C", HERE])
        os.makedirs(os.path.dirname(LOG), exist_ok=True)
        cls.port = free_port()
        cls.log = open(LOG, "w")
        cls.proc = subprocess.Popen([SIM, "--port", str(cls.port)] + shlex.split(os.environ.get("SIM_ARGS", "")), stdin=subprocess.DEVNULL, stdout=cls.log,
                                    stderr=subprocess.STDOUT)
        t0 = time.time()
        while True:
            try:
                cls.get_status()
                break
            except OSError:
                if cls.proc.poll() is not None or time.time() - t0 > 10:
                    raise RuntimeError("simulator did not start, see " + LOG)
                time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(5)
        cls.log.close()

    # ------------------------------------------------------------------------------------------------ HTTP helpers
    @classmethod
    def get(cls, path, timeout=3):
        """GET path; returns (HTTP code, body text). A 409 is an answer, not an error."""
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{cls.port}{path}", timeout=timeout) as r:
                return r.status, r.read().decode()
        except urllib.error.HTTPError as e:
            return e.code, e.read().decode()

    @classmethod
    def get_status(cls):
        code, body = cls.get("/status")
        assert code == 200, (code, body)
        return json.loads(body)

    def sim_state(self):
        code, body = self.get("/sim/state")
        self.assertEqual(code, 200, body)
        return json.loads(body)

    def sim_set(self, **knobs):
        q = "&".join(f"{k}={v}" for k, v in knobs.items())
        code, body = self.get("/sim/set?" + q)
        self.assertEqual(code, 200, body)
        return json.loads(body)

    def wait_for(self, pred, timeout, what, poll=0.05, fail=None):
        """poll /status until pred(status) holds; fail early when fail(status) holds. Returns the last status."""
        t0 = time.time()
        while True:
            s = self.get_status()
            if pred(s):
                return s
            if fail and fail(s):
                self.fail(f"while waiting for {what}: {s}")
            if time.time() - t0 > timeout:
                self.fail(f"timed out after {timeout} s waiting for {what}; last status {s}")
            time.sleep(poll)

    def hold(self, c, seconds, period=0.15, stop_on_refusal=False):
        """repeat a run key like the web page does (every 150 ms, ms=350). Returns [(code, body)], time of last send."""
        out, t_end, t_last = [], time.time() + seconds, None
        while time.time() < t_end:
            code, body = self.get(f"/key?c={urllib.request.quote(c)}&ms=350")
            out.append((code, body)); t_last = time.time()
            if stop_on_refusal and code == 409:
                break
            time.sleep(period)
        return out, t_last

    def assertRefused(self, path, reason_prefix):
        code, body = self.get(path)
        self.assertEqual(code, 409, f"{path} should be refused, got {code} {body!r}")
        self.assertTrue(body.startswith("refused: " + reason_prefix), f"{path}: {body!r}")

    # ------------------------------------------------------------------------------------------------ tests
    def test_01_status_fields_and_idle(self):
        s = self.get_status()
        for k in STATUS_FIELDS:
            self.assertIn(k, s)
        self.assertTrue(s["fw"].startswith("kedge-dart-v4"))
        self.assertEqual(s["phase"], "IDLE")
        self.assertIsNone(s["fault"])
        self.assertEqual(s["seq"], "idle")
        self.assertEqual(sorted(s["motors"]), sorted(MOTORS))
        self.assertTrue(all(v == 0 for v in s["motors"].values()))
        self.assertTrue(s["ina_ok"] and s["imu_ok"])
        self.assertAlmostEqual(s["rail_V"], 7.6, delta=0.01)
        self.assertEqual(s["reset"], "poweron")
        self.assertIsNone(s["t_launch_ms"])
        self.assertIsNone(s["t_land_ms"])
        self.assertAlmostEqual(s["accel_g"], 1.0, delta=0.05)

    def test_02_key_hold_runs_and_deadman_stops(self):
        # hold "kick out": every status read during the hold shows the kick running
        t_end, seen = time.time() + 1.2, []
        while time.time() < t_end:
            code, body = self.get("/key?c=2&ms=350")
            self.assertEqual(code, 200, body)
            t_last = time.time()
            seen.append(self.get_status()["motors"]["kick"])
            time.sleep(0.15)
        self.assertTrue(all(k == 1 for k in seen), seen)
        st = self.sim_state()
        self.assertEqual(st["motors"]["kick"]["dir"], 1)
        self.assertGreater(st["kick_pos"], 800)              # about 1.2 s of travel so far
        # repeats stopped: the deadman (350 ms after the last key) stops it
        s = self.wait_for(lambda s: s["motors"]["kick"] == 0, 1.5, "deadman stop", poll=0.02)
        dt = time.time() - t_last
        self.assertTrue(0.25 <= dt <= 0.6, f"kick stopped {dt:.3f} s after the last key (deadman is 0.35 s)")
        self.assertEqual(self.sim_state()["motors"]["kick"]["dir"], 0)
        # hold "kick back" into the stowed stop: the stall stop catches it, then the held key is refused
        replies, _ = self.hold("@", 5.0, stop_on_refusal=True)
        self.assertEqual(replies[-1][0], 409, replies[-3:])
        self.assertTrue(replies[-1][1].startswith("refused: stalled"), replies[-1])
        s = self.get_status()
        self.assertEqual(s["motors"]["kick"], 0)
        self.assertIn("stalled: kick", s["event"])
        self.assertEqual(self.sim_state()["kick_pos"], 0)
        time.sleep(0.6)                                      # "release" the key

    def test_03_stop_disarm0_keeps_armed(self):
        code, body = self.get("/arm")
        self.assertEqual(code, 200, body)
        s = self.get_status()
        self.assertTrue(s["armed"]); self.assertEqual(s["phase"], "ARMED")
        code, _ = self.get("/stop?disarm=0")
        self.assertEqual(code, 200)
        s = self.get_status()
        self.assertTrue(s["armed"], s); self.assertEqual(s["phase"], "ARMED")
        code, _ = self.get("/stop")
        self.assertEqual(code, 200)
        s = self.get_status()
        self.assertFalse(s["armed"]); self.assertEqual(s["phase"], "IDLE")

    def test_04_ina_dead_refuses_auto_and_recovers(self):
        self.sim_set(ina_dead=1)
        s = self.wait_for(lambda s: not s["ina_ok"], 1.0, "ina_ok false")
        self.assertEqual(s["rail_V"], -1.0)
        self.assertRefused("/auto", "no current sensor")
        self.assertRefused("/release", "no current sensor")
        self.sim_set(ina_dead=0)                             # powers up uncalibrated: the sketch must set it up again
        self.wait_for(lambda s: s["ina_ok"], 1.0, "ina_ok true again")
        time.sleep(0.3)
        st = self.sim_state()
        self.assertEqual(st["ina"]["cal"], 4096, "sketch did not re-calibrate the INA219")
        self.assertEqual(st["ina"]["config"], 0x39D7)

    def test_05_drill_jam_in_release_faults(self):
        self.sim_set(drill_jam=1)
        try:
            code, body = self.get("/release")
            self.assertEqual(code, 200, body)
            s = self.wait_for(lambda s: s["phase"] == "FAULT", 8.0, "DRILL_JAM fault (after 5 rocks, v4.1)")
            self.assertEqual(s["fault"], "DRILL_JAM")
            self.assertIn("rocked", s["event"])
            self.assertTrue(all(v == 0 for v in s["motors"].values()), s)
            self.assertTrue(all(m["dir"] == 0 for m in self.sim_state()["motors"].values()))
        finally:
            self.sim_set(drill_jam=0)
        self.get("/stop")
        s = self.get_status()
        self.assertEqual(s["phase"], "IDLE"); self.assertIsNone(s["fault"])

    def test_06_low_rail_faults_a_running_motor(self):
        self.sim_set(rail_v=5.0)
        try:
            self.hold("1", 1.2)
            s = self.wait_for(lambda s: s["fault"] is not None, 1.0, "LOW_RAIL fault")
            self.assertEqual(s["fault"], "LOW_RAIL")
            self.assertAlmostEqual(s["rail_V"], 5.0, delta=0.01)
        finally:
            self.sim_set(rail_v=7.6)
        self.wait_for(lambda s: all(v == 0 for v in s["motors"].values()), 1.0, "motors off")
        self.get("/stop")
        self.hold("!", 1.8)                                  # drive the pods back up (the dart refuses to arm with legs out)
        self.get("/stop")
        self.assertEqual(self.get_status()["phase"], "IDLE")

    def test_07_mission_arm_launch_flight_anchoring_anchored(self):
        phases = []
        def note(s):
            if not phases or phases[-1] != s["phase"]:
                phases.append(s["phase"])
            return s
        note(self.get_status())
        code, body = self.get("/arm")
        self.assertEqual(code, 200, body)
        note(self.get_status())
        code, _ = self.get("/sim/launch")
        self.assertEqual(code, 200)
        note(self.wait_for(lambda s: s["phase"] == "FLIGHT", 3.0, "FLIGHT", poll=0.02))
        # refusals in flight
        self.assertRefused("/auto", "in flight")
        self.assertRefused("/key?c=2&ms=350", "in flight")
        self.assertRefused("/release", "in flight")
        # landing starts the anchor sequence by itself
        s = note(self.wait_for(lambda s: s["phase"] == "ANCHORING", 5.0, "ANCHORING", fail=lambda s: s["phase"] == "FAULT"))
        self.assertTrue(s["landed"])
        self.assertRefused("/release", "busy")
        self.assertRefused("/key?c=1&ms=350", "busy")
        self.assertRefused("/flightlog", "busy")
        code, _ = self.get("/auto")                          # a repeated /auto while anchoring is accepted, no restart
        self.assertEqual(code, 200)
        s = self.wait_for(lambda s: note(s)["phase"] == "ANCHORED", 40.0, "ANCHORED", poll=0.2,
                          fail=lambda s: s["phase"] == "FAULT" or s["seq"] == "aborted")
        self.assertEqual(s["seq"], "anchored")
        self.assertEqual(phases, ["IDLE", "ARMED", "FLIGHT", "ANCHORING", "ANCHORED"])
        self.assertIsInstance(s["t_launch_ms"], int); self.assertIsInstance(s["t_land_ms"], int)
        self.assertTrue(150 < s["t_land_ms"] - s["t_launch_ms"] < 2000, s)
        st = self.sim_state()
        self.assertEqual(st["kick_pos"], 4300); self.assertEqual(st["deploy_pos"], 1400)
        self.assertTrue(all(m["dir"] == 0 for m in st["motors"].values()))

    def test_08_flightlog_after_landing(self):
        s = self.wait_for(lambda s: s["log_frozen"], 5.0, "log_frozen")
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        conn.request("GET", "/flightlog")
        r = conn.getresponse()
        self.assertEqual(r.status, 200)
        self.assertEqual(r.getheader("Transfer-Encoding"), "chunked")
        j = json.loads(r.read().decode())
        conn.close()
        self.assertEqual(j["hz"], 50)
        self.assertTrue(j["frozen"])
        self.assertEqual(j["landing"], "impact")
        self.assertEqual(j["t_launch_ms"], s["t_launch_ms"]); self.assertEqual(j["t_land_ms"], s["t_land_ms"])
        self.assertEqual(j["fields"], ["t_ms", "qw", "qx", "qy", "qz", "ax", "ay", "az"])
        smp = j["samples"]
        self.assertEqual(j["n"], len(smp))
        self.assertTrue(all(len(x) == 8 for x in smp))
        ts = [x[0] for x in smp]
        self.assertEqual(ts, sorted(ts))
        dts = [b - a for a, b in zip(ts, ts[1:])]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) * 1000
        self.assertTrue(18 <= statistics.median(dts) <= 22, statistics.median(dts))
        self.assertTrue(45 <= rate <= 55, f"{rate:.1f} Hz")
        # window: 2 s before launch to 3 s after landing
        self.assertTrue(abs(ts[0] - (j["t_launch_ms"] - 2000)) <= 25, (ts[0], j["t_launch_ms"]))
        self.assertTrue(abs(ts[-1] - (j["t_land_ms"] + 3000)) <= 25, (ts[-1], j["t_land_ms"]))
        # the launch push and the impact read flat at the 8 g clip; the dart pitched in flight
        self.assertTrue(any(abs(x[7]) > 78 for x in smp))
        self.assertTrue(any(abs(x[3]) > 0.15 for x in smp))  # qy: pitch about 25 degrees during the flight

    def test_09_release_ends_released_idle(self):
        code, body = self.get("/release")
        self.assertEqual(code, 200, body)
        self.assertEqual(self.get_status()["phase"], "RELEASING")
        self.assertRefused("/auto", "busy")
        s = self.wait_for(lambda s: s["seq"] == "released", 40.0, "released", poll=0.2,
                          fail=lambda s: s["phase"] == "FAULT" or s["seq"] == "aborted")
        self.assertEqual(s["phase"], "IDLE")
        st = self.sim_state()
        self.assertEqual(st["kick_pos"], 0); self.assertEqual(st["deploy_pos"], 0)
        self.assertTrue(all(m["dir"] == 0 for m in st["motors"].values()))

    def test_10_old_jetson_client(self):
        sys.path.insert(0, JETSON_CLIENT_DIR)
        try:
            import kedge_client
        finally:
            sys.path.remove(JETSON_CLIENT_DIR)
        d = kedge_client.Dart(host=f"127.0.0.1:{self.port}", timeout=2.0)
        s = d.status()
        self.assertEqual(s["phase"], "IDLE")
        for k in ("rail_V", "current_mA", "roll", "pitch", "seq", "event"):   # the fields the client prints
            self.assertIn(k, s)
        self.assertIn("rail", d.key("i"))
        seen = []
        t = threading.Thread(target=d.hold, args=("4", 1.2))
        t.start()
        while t.is_alive():
            seen.append(self.get_status()["motors"]["foot"])
            time.sleep(0.1)
        t.join()
        self.assertIn(1, seen)
        self.assertGreaterEqual(seen.count(1), 6, seen)       # running for (nearly) the whole hold
        s = self.get_status()                                # hold() ends with /stop
        self.assertEqual(s["motors"]["foot"], 0)
        self.assertEqual(s["event"], "stop")

    def test_11_imu_stays_fresh_with_realistic_i2c_time(self):
        """Regression (fixed 3 Nov): imu_ok must stay true while the BNO085 is sending data.

        readImu(now) gets `now` from tickTask, then imuEvent() (inside sh2_service) stores lastImuMs = millis(). When
        the SH-2 transfer takes long enough for millis() to tick over (on the hardware, an I2C read of a report packet
        at 400 kHz takes about 0.5 to 1 ms), lastImuMs > now and `now - lastImuMs < 1000` wraps (uint32_t) to false.
        Fixed: imuEvent now uses the tick's time and the freshness test is signed.
        """
        self.sim_set(imu_xfer_us=1000)
        try:
            time.sleep(0.5)
            oks = []
            for _ in range(10):
                oks.append(self.get_status()["imu_ok"]); time.sleep(0.1)
            code, body = self.get("/arm?auto=0")
            self.assertTrue(all(oks), f"imu_ok samples with 1 ms SH-2 transfers: {oks}")
            self.assertEqual(code, 200, body)
        finally:
            self.sim_set(imu_xfer_us=0)
            self.get("/arm?on=0")


    def test_12_no_ground_faults_and_release_stows(self):
        # dart on its back: the kick swings through air to its end stop; NO_GROUND, augers never run; /release stows
        self.get("/stop"); time.sleep(0.3)
        self.sim_set(kick_pos=0, deploy_pos=0, no_ground=1)
        try:
            code, body = self.get("/auto")
            self.assertEqual(code, 200, body)
            s = self.wait_for(lambda s: s["phase"] == "FAULT", 15.0, "NO_GROUND fault")
            self.assertEqual(s["fault"], "NO_GROUND")
            self.assertTrue(all(v == 0 for v in s["motors"].values()), s)
            self.assertEqual(self.sim_state()["deploy_pos"], 0, "pods never moved")
            self.get("/auto")                        # a retry faults again (never anchors in air)
            time.sleep(1.5)
            self.assertEqual(self.get_status()["fault"], "NO_GROUND")
            self.assertEqual(self.sim_state()["deploy_pos"], 0)
            code, body = self.get("/release")
            self.assertEqual(code, 200, body)
            s = self.wait_for(lambda s: s["seq"] == "released", 40.0, "released", poll=0.3)
            self.assertEqual(self.sim_state()["kick_pos"], 0)
        finally:
            self.sim_set(no_ground=0)
            self.get("/stop")

    def test_13_v41_video_orientation_battery_short_flight(self):
        # v4.1: optional video switch on D10, /auto refused on its side unless force=1, battery critical at rest,
        # a 0.33 s flight (the real launcher at its low aim) still lands by impact
        self.get("/stop"); time.sleep(0.3)
        self.sim_set(kick_pos=0, deploy_pos=0)
        self.assertEqual(self.sim_state()["video_pin"], 1, "video on after boot")
        code, _ = self.get("/video?on=0")
        self.assertEqual(code, 200)
        self.assertEqual(self.sim_state()["video_pin"], 0)
        self.assertFalse(self.get_status()["video"])
        code, _ = self.get("/video")
        self.assertEqual(code, 400, "missing on")
        self.get("/video?on=1")
        self.assertEqual(self.sim_state()["video_pin"], 1)
        try:
            self.get("/sim/trace?s=600:9.81:0:0")      # on its side
            self.wait_for(lambda s: s["orient"] == "side" and s["tilt_deg"] > 85, 3.0, "orient side, tilt about 90")
            code, body = self.get("/auto")
            self.assertEqual(code, 409, body)
            self.assertIn("not upright: on its side", body)
            self.assertNotEqual(self.get_status()["phase"], "ANCHORING", "a refused /auto moves nothing")
            code, body = self.get("/auto?force=1")
            self.assertEqual(code, 200, body)
            self.wait_for(lambda s: s["phase"] == "ANCHORING", 2.0, "forced auto runs")
            self.get("/stop"); time.sleep(0.3)
            self.get("/sim/trace?s=600:0:0:-9.81")     # upside down
            self.wait_for(lambda s: s["orient"] == "inverted", 3.0, "orient inverted")
        finally:
            self.get("/sim/trace?s=600:0:0:9.81")
            self.wait_for(lambda s: s["orient"] == "upright", 3.0, "upright again")
        # release what the forced auto moved (kick only), so later tests start stowed
        self.sim_set(kick_pos=0, deploy_pos=0)
        try:
            self.sim_set(rail_v=6.3)
            s = self.wait_for(lambda s: s["batt"] == "critical", 4.0, "battery critical at rest")
            self.assertFalse(s["video"], "critical switches the video off")
            self.assertEqual(self.sim_state()["video_pin"], 0)
            code, body = self.get("/auto")
            self.assertEqual(code, 409, body)
            self.assertIn("battery low", body)
        finally:
            self.sim_set(rail_v=7.6)
            self.wait_for(lambda s: s["batt"] == "ok", 4.0, "battery ok again")
            self.get("/video?on=1")
        # short flight, log only
        code, body = self.get("/arm?on=1&auto=0")
        self.assertEqual(code, 200, body)
        self.get("/sim/launch?push_ms=45&flight_ms=330&impact=1")
        s = self.wait_for(lambda s: s["phase"] == "LANDED", 6.0, "landed after a 0.33 s flight")
        self.assertIsNotNone(s["t_land_ms"])
        time.sleep(3.2)                                 # the log freezes 3 s after the landing
        code, body = self.get("/flightlog", timeout=5)
        self.assertEqual(code, 200, body[:200])
        self.assertEqual(json.loads(body)["landing"], "impact")
        self.get("/stop")

    def test_14_v41_milling_deploy_in_soil(self):
        # the pods stall in soil from 500 of 1400: the anchor mills them down to the stop instead of stopping there
        self.get("/stop"); time.sleep(0.3)
        self.sim_set(kick_pos=0, deploy_pos=0, soil_at=500)
        try:
            code, body = self.get("/auto?force=1")
            self.assertEqual(code, 200, body)
            self.wait_for(lambda s: s["phase"] in ("ANCHORED", "FAULT"), 50.0, "anchored", poll=0.3)
            s = self.get_status()
            self.assertEqual(s["phase"], "ANCHORED", s)
            self.assertGreaterEqual(self.sim_state()["deploy_pos"], 1390, "pods milled down to the stop")
            self.assertIn("milling", open(LOG).read())
        finally:
            self.sim_set(soil_at=-1)
            self.get("/stop")
            self.sim_set(kick_pos=0, deploy_pos=0)

    def test_15_v42_drill_sign_and_deploy_guard(self):
        # v32 as built: direct drive, key 3 (drills in) turns motors and sleeves the "in" way (DRILL_SIGN +1)
        self.get("/stop"); time.sleep(0.3)
        self.sim_set(kick_pos=0, deploy_pos=0)
        self.get("/key?c=3&ms=400")
        time.sleep(0.1)
        d = self.sim_state()["motors"]["drills"]
        self.assertEqual((d["dir"], d["motor_dir"]), (1, 1), d)
        self.assertEqual(self.get_status()["motors"]["drills"], 1)
        self.get("/stop"); time.sleep(0.3)
        # deploy guard: drills jammed (700 mA, over 67 % of the 700 mA stall) while the pods go down: ANCHORED, depth_limited
        self.sim_set(drill_jam=1)
        try:
            code, body = self.get("/auto?force=1")
            self.assertEqual(code, 200, body)
            s = self.wait_for(lambda s: s["phase"] in ("ANCHORED", "FAULT"), 30.0, "anchored", poll=0.3)
            self.assertEqual(s["phase"], "ANCHORED", s)
            self.assertIn("depth_limited", s["warnings"])
            self.assertLess(self.sim_state()["deploy_pos"], 1400, "pods stopped part way")
        finally:
            self.sim_set(drill_jam=0)
            self.get("/stop")
            self.sim_set(kick_pos=0, deploy_pos=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
