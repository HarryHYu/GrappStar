// GrappStar operator page. Vanilla JS, no dependencies, works offline.
// Talks to the rover only through /api/status, /api/action/<name>, /stream, /snapshot, /artifact.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 300;
const POLL_MAX_MS = 3000;
const REPEAT_MS = 150;     // hold-to-run repeat period
const JOG_MS = 300;        // each jog command runs this long: releasing stops within JOG_MS (deadman)
const DART_MS = 350;
const ESTOP_HOLD_MS = 1000;

// Default mission steps (used when status.mission.steps is not provided by the App).
const DEFAULT_STEPS = [
  "PREFLIGHT", "LAUNCH_MODE", "AIM", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "ANCHOR",
  "REEL_MODE", "WRAP", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "NEUTRAL", "DONE",
];
const BASE_FIELDS = ["state", "mode", "drum_pos", "pitch_deg", "cock", "wraps", "line_m", "tension_n", "amps", "vbat", "fault"];
const DART_FIELDS = ["phase", "seq", "rail_V", "current_mA", "event"];
const DART_KEYS = [["1", "deploy down"], ["!", "deploy up"], ["2", "kick out"], ["@", "kick back"],
  ["3", "drills in"], ["#", "drills out"], ["4", "foot in"], ["$", "foot out"]];
const SAFE_CHOICE = /^(cancel|abort|no|back|stop|wait|hold|safe)/i;

let status = null;
let lastTarget = null;          // {u, v} of the last tap, used when status.target has no u, v

// ---------------------------------------------------------------- helpers
function el(tag, attrs, text) {
  const e = document.createElement(tag);
  for (const k in attrs || {}) e.setAttribute(k, attrs[k]);
  if (text !== undefined) e.textContent = text;
  return e;
}
function fmt(v) {
  if (v === null || v === undefined || v === "") return "-";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(Math.abs(v) < 10 ? 2 : 1);
  if (typeof v === "boolean") return v ? "yes" : "no";
  if (typeof v === "object") return JSON.stringify(v);
  return String(v);
}
function pick(obj, key) {
  // Telemetry may be flat or nested under telemetry / tlm / status / last.
  if (!obj || typeof obj !== "object") return undefined;
  if (key in obj) return obj[key];
  for (const sub of ["telemetry", "tlm", "status", "last", "data"]) {
    if (obj[sub] && typeof obj[sub] === "object" && key in obj[sub]) return obj[sub][key];
  }
  return undefined;
}
function linkOk(x) {
  // Accepts a bool, or an object with ok / connected / up / alive, or a nested link field.
  if (x === null || x === undefined) return null;
  if (typeof x === "boolean") return x;
  if (typeof x !== "object") return !!x;
  if ("link" in x) return linkOk(x.link);
  for (const k of ["ok", "connected", "up", "alive", "online"]) if (k in x) return !!x[k];
  return null;
}
let toastTimer = 0;
function toast(msg, bad) {
  const t = $("toast");
  t.textContent = msg;
  t.classList.toggle("bad", !!bad);
  t.classList.remove("hidden");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add("hidden"), bad ? 6000 : 2500);
}

// ---------------------------------------------------------------- actions
async function action(name, args, quiet) {
  try {
    const r = await fetch("/api/action/" + encodeURIComponent(name), {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(args || {}),
    });
    let j;
    try { j = await r.json(); } catch (e) { j = { ok: false, msg: "HTTP " + r.status }; }
    if (!j.ok) toast(name + ": " + (j.msg || "failed"), true);
    else if (!quiet && j.msg) toast(j.msg, false);
    return j;
  } catch (e) {
    toast(name + ": no link to rover", true);
    return { ok: false, msg: "network" };
  }
}

// ---------------------------------------------------------------- STOP / ESTOP
(function setupStop() {
  const b = $("btn-stop");
  let timer = 0, fired = false;
  b.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    fired = false;
    action("stop", {});             // stop immediately on touch, not on release
    stopHolds();
    b.classList.add("armed");
    timer = setTimeout(() => { fired = true; action("estop", {}); toast("ESTOP sent", true); }, ESTOP_HOLD_MS);
  });
  const end = () => { clearTimeout(timer); b.classList.remove("armed"); };
  b.addEventListener("pointerup", end);
  b.addEventListener("pointerleave", end);
  b.addEventListener("pointercancel", end);
  b.addEventListener("click", (e) => e.preventDefault());
  b.addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); action("stop", {}); } });
  $("btn-estop").addEventListener("click", () => { stopHolds(); action("estop", {}); toast("ESTOP sent", true); });
})();

// ---------------------------------------------------------------- plain action buttons
document.querySelectorAll("button[data-action]").forEach((b) => {
  b.addEventListener("click", () => {
    const c = b.getAttribute("data-confirm");
    if (c && !confirm(c)) return;
    let args = {};
    try { args = JSON.parse(b.getAttribute("data-args") || "{}"); } catch (e) { /* keep {} */ }
    action(b.getAttribute("data-action"), args);
  });
});
$("btn-aim-range").addEventListener("click", () => {
  const v = parseFloat($("in-range").value);
  if (!isFinite(v) || v <= 0) return toast("Enter a range in m", true);
  action("aim", { range_m: v });
});
$("btn-aim-deg").addEventListener("click", () => {
  const v = parseFloat($("in-deg").value);
  if (!isFinite(v)) return toast("Enter a pitch in degrees", true);
  action("aim", { deg: v });
});
$("btn-aim-target").addEventListener("click", () => {
  const t = status && status.target;
  if (!t || typeof t.range_m !== "number") return toast("No target set", true);
  action("aim", { range_m: t.range_m });
});

// ---------------------------------------------------------------- hold-to-run (jog + dart keys)
const holds = new Map();     // id -> {kind, args}
let holdTimer = 0;
function sendHold(h) {
  if (h.kind === "jog") action("jog", h.args, true);
  else action("dart_key", h.args, true);
}
function startHold(id, h) {
  if (holds.has(id)) return;
  holds.set(id, h);
  sendHold(h);
  if (!holdTimer) holdTimer = setInterval(() => { for (const x of holds.values()) sendHold(x); }, REPEAT_MS);
}
function endHold(id) {
  const h = holds.get(id);
  if (!h) return;
  holds.delete(id);
  if (h.kind === "dart" && ![...holds.values()].some((x) => x.kind === "dart")) action("dart_stop", {}, true);
  if (!holds.size) { clearInterval(holdTimer); holdTimer = 0; }
}
function stopHolds() {
  const hadDart = [...holds.values()].some((x) => x.kind === "dart");
  holds.clear();
  clearInterval(holdTimer); holdTimer = 0;
  document.querySelectorAll("button.hold.active").forEach((b) => b.classList.remove("active"));
  if (hadDart) action("dart_stop", {}, true);
}
function bindHold(b, id, h) {
  b.tabIndex = -1;
  b.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    try { b.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
    b.classList.add("active");
    startHold(id, h);
  });
  const up = () => { b.classList.remove("active"); endHold(id); };
  b.addEventListener("pointerup", up);
  b.addEventListener("pointercancel", up);
  b.addEventListener("lostpointercapture", up);
  b.addEventListener("contextmenu", (e) => e.preventDefault());
}
document.querySelectorAll("button[data-jog]").forEach((b) => {
  const motor = b.getAttribute("data-jog"), dir = parseInt(b.getAttribute("data-dir"), 10);
  bindHold(b, "jog" + motor + dir, { kind: "jog", args: { motor: motor, dir: dir, ms: JOG_MS } });
});
(function dartButtons() {
  const box = $("dart-keys");
  for (const [c, name] of DART_KEYS) {
    const b = el("button", { class: "hold", title: "key " + c }, name + " (" + c + ")");
    box.appendChild(b);
    bindHold(b, "btn" + c, { kind: "dart", args: { c: c, ms: DART_MS } });
  }
})();
function typing(e) {
  const t = e.target;
  return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.isContentEditable);
}
addEventListener("keydown", (e) => {
  if (typing(e)) return;
  if (e.key === " ") { e.preventDefault(); if (!e.repeat) { stopHolds(); action("stop", {}); } return; }
  if ($("manual").open && DART_KEYS.some(([c]) => c === e.key)) {
    e.preventDefault();
    startHold("key" + e.code, { kind: "dart", args: { c: e.key, ms: DART_MS } });    // keyed by physical key
  }
});
addEventListener("keyup", (e) => { if (holds.has("key" + e.code)) endHold("key" + e.code); });
function lost() { if (holds.size) stopHolds(); }
addEventListener("blur", lost);
addEventListener("pagehide", lost);
document.addEventListener("visibilitychange", () => { if (document.hidden) lost(); });

// ---------------------------------------------------------------- videos
function startVideo(img) {
  const cam = img.getAttribute("data-cam");
  let n = 0;
  const load = () => { img.src = "/stream/" + cam + ".mjpg?r=" + (n++); };
  img.addEventListener("error", () => setTimeout(load, 2000));
  load();
}
document.querySelectorAll("img[data-cam]").forEach(startVideo);

// Rectangle of the displayed picture inside an object-fit: contain <img>, in element pixels.
function imageRect(img) {
  const W = img.clientWidth, H = img.clientHeight;
  const nw = img.naturalWidth || W, nh = img.naturalHeight || H;
  const s = Math.min(W / nw, H / nh);
  const w = nw * s, h = nh * s;
  return { x: (W - w) / 2, y: (H - h) / 2, w: w, h: h };
}
$("cam-rover").addEventListener("click", (e) => {
  const img = e.currentTarget, r = imageRect(img), box = img.getBoundingClientRect();
  const u = (e.clientX - box.left - r.x) / r.w, v = (e.clientY - box.top - r.y) / r.h;
  if (u < 0 || u > 1 || v < 0 || v > 1) return;    // tap on the letterbox bars
  lastTarget = { u: u, v: v };
  drawCrosshair();
  action("target_click", { cam: "rover", u: +u.toFixed(4), v: +v.toFixed(4) }, true);
});
function drawCrosshair() {
  const t = status && status.target;
  let p = null;
  if (t && typeof t.u === "number" && typeof t.v === "number" && (!t.cam || t.cam === "rover")) p = t;
  else if (lastTarget && t) p = lastTarget;
  else if (lastTarget && !status) p = lastTarget;
  const x = $("xhair");
  if (!p) { x.classList.add("hidden"); return; }
  const img = $("cam-rover"), r = imageRect(img);
  x.style.left = (r.x + p.u * r.w) + "px";
  x.style.top = (r.y + p.v * r.h) + "px";
  x.classList.remove("hidden");
}
addEventListener("resize", drawCrosshair);
$("cam-rover").addEventListener("load", drawCrosshair);

function renderTarget(t) {
  const box = $("target");
  box.textContent = "";
  if (!t || typeof t.range_m !== "number") {
    box.textContent = "No target. Tap the rover view or use Auto target.";
    return;
  }
  const b = typeof t.bearing_deg === "number" ? t.bearing_deg : 0;
  const line1 = el("div", {}, "Range " + t.range_m.toFixed(1) + " m, bearing " + (b >= 0 ? "+" : "") + b.toFixed(1) + " deg");
  const turn = Math.abs(b) < 1 ? "Facing target (within 1 deg)" :
    "Turn rover " + Math.abs(b).toFixed(0) + " degrees " + (b > 0 ? "RIGHT" : "LEFT");
  box.appendChild(line1);
  box.appendChild(el("div", { class: "turn" }, turn));
  const s = t.solution;
  if (s) {
    const reach = s.reachable;
    const d = el("div", { class: reach ? "yes" : "no" },
      (reach ? "Reachable" : "NOT reachable") +
      (typeof s.pitch_deg === "number" ? ", pitch " + s.pitch_deg.toFixed(1) + " deg" : "") +
      (typeof s.line_out_m === "number" ? ", line " + s.line_out_m.toFixed(1) + " m" : "") +
      (s.note ? " (" + s.note + ")" : ""));
    box.appendChild(d);
  }
}

// ---------------------------------------------------------------- prompt card
let shownPrompt = null;    // id of the prompt on screen
let answered = null;       // id answered, waiting for the mission to move on
function renderPrompt(p) {
  const sec = $("prompt");
  if (!p) { sec.classList.add("hidden"); shownPrompt = null; return; }
  const key = p.id + "|" + p.text + "|" + (p.choices || []).join(",");
  if (shownPrompt === key) return;
  shownPrompt = key;
  answered = null;
  sec.classList.toggle("danger", !!p.danger);
  $("prompt-text").textContent = p.text || "";
  const box = $("prompt-choices");
  box.textContent = "";
  for (const choice of p.choices || []) {
    const dangerous = !!p.danger && !SAFE_CHOICE.test(choice);
    const b = el("button", { class: dangerous ? "danger-choice" : "" }, choice);
    b.addEventListener("click", async () => {
      if (answered === p.id) return;
      if (dangerous && !confirm("Are you sure: " + choice + "?\n\n" + (p.text || ""))) return;
      answered = p.id;
      box.querySelectorAll("button").forEach((x) => (x.disabled = true));
      const r = await action("answer", { prompt_id: p.id, choice: choice });
      if (!r.ok) { answered = null; box.querySelectorAll("button").forEach((x) => (x.disabled = false)); }
    });
    box.appendChild(b);
  }
  sec.classList.remove("hidden");
}

// ---------------------------------------------------------------- render status
function setPill(id, ok, label) {
  const p = $(id);
  p.classList.toggle("ok", ok === true);
  p.classList.toggle("bad", ok === false);
  p.textContent = label + (ok === true ? " OK" : ok === false ? " DOWN" : " ?");
}
function fillTable(id, src, fields) {
  const tb = $(id).tBodies[0];
  if (tb.rows.length !== fields.length) {
    tb.textContent = "";
    for (const f of fields) { const tr = el("tr"); tr.appendChild(el("td", {}, f)); tr.appendChild(el("td", {}, "-")); tb.appendChild(tr); }
  }
  fields.forEach((f, i) => {
    const v = pick(src, f);
    const row = tb.rows[i];
    row.cells[1].textContent = fmt(v);
    if (f === "fault") row.classList.toggle("fault", !!v && v !== "none" && v !== "NONE" && v !== 0 && v !== "0");
  });
}
function stepName(h) {
  if (typeof h === "string") return h;
  if (h && typeof h === "object") return h.state || h.step || h.name || "";
  return "";
}
function renderMission(m) {
  m = m || {};
  $("m-state").textContent = fmt(m.state);
  $("m-step").textContent = fmt(m.step);
  const err = m.error || m.last_error;
  $("m-error").textContent = err ? "Error: " + fmt(err) : "";
  $("m-error").classList.toggle("hidden", !err);
  const steps = Array.isArray(m.steps) && m.steps.length ? m.steps.map(stepName) : DEFAULT_STEPS;
  const done = new Set((m.history || []).map(stepName).map((s) => String(s).toUpperCase()));
  const cur = String(m.state || "").toUpperCase();
  const ol = $("m-steps");
  if (ol.children.length !== steps.length || ol.dataset.k !== steps.join(",")) {
    ol.textContent = "";
    ol.dataset.k = steps.join(",");
    for (const s of steps) ol.appendChild(el("li", {}, String(s).replace(/_/g, " ").toLowerCase()));
  }
  steps.forEach((s, i) => {
    const S = String(s).toUpperCase();
    ol.children[i].classList.toggle("cur", S === cur);
    ol.children[i].classList.toggle("done", S !== cur && done.has(S));
  });
  renderPrompt(m.prompt || null);
}
let lzKey = null, moKey = null, bust = 0;
function renderResults(res) {
  res = res || {};
  const tr = res.terrain;
  const k1 = tr ? JSON.stringify(tr) : null;
  if (k1 !== lzKey) {
    lzKey = k1; bust++;
    const v = $("lz-verdict"), img = $("lz-img");
    v.className = "verdict";
    $("lz-reasons").textContent = "";
    if (!tr) { v.textContent = "No report yet"; $("lz-info").textContent = ""; img.classList.add("hidden"); }
    else {
      v.textContent = (tr.verdict || "?").replace("_", " ");
      if (tr.verdict) v.classList.add(tr.verdict);
      $("lz-info").textContent = "Class " + fmt(tr.cls) + ", score " + fmt(tr.score) +
        (typeof tr.slope_deg === "number" ? ", slope " + tr.slope_deg.toFixed(0) + " deg" : "");
      for (const r of tr.reasons || []) $("lz-reasons").appendChild(el("li", {}, String(r)));
      img.src = "/artifact/landing.png?v=" + bust + "_" + Date.now();
      img.onerror = () => img.classList.add("hidden");
      img.onload = () => img.classList.remove("hidden");
    }
  }
  const mo = res.mosaic;
  const k2 = mo ? JSON.stringify(mo) : null;
  if (k2 !== moKey) {
    moKey = k2; bust++;
    const img = $("mo-img");
    if (!mo) { $("mo-info").textContent = "No mosaic yet"; img.classList.add("hidden"); }
    else {
      const used = Array.isArray(mo.used) ? mo.used.length : mo.used;
      const rej = Array.isArray(mo.rejected) ? mo.rejected.length : mo.rejected;
      $("mo-info").textContent = "Frames used " + fmt(used) + ", rejected " + fmt(rej);
      img.src = "/artifact/mosaic.png?v=" + bust + "_" + Date.now();
      img.onerror = () => img.classList.add("hidden");
      img.onload = () => img.classList.remove("hidden");
    }
  }
}
function render(st) {
  status = st;
  const cams = st.cams || {};
  setPill("ln-base", linkOk(st.base), "BASE");
  setPill("ln-dart", linkOk(st.dart), "DART");
  setPill("ln-rover", cams.rover ? linkOk(cams.rover) : false, "ROVER CAM");
  setPill("ln-dartcam", cams.dart ? linkOk(cams.dart) : false, "DART CAM");
  fillTable("t-base", st.base, BASE_FIELDS);
  fillTable("t-dart", st.dart, DART_FIELDS);
  renderMission(st.mission);
  renderTarget(st.target);
  drawCrosshair();
  renderResults(st.results);
  if (typeof st.t === "number") $("clock").textContent = "rover time " + new Date(st.t * 1000).toLocaleTimeString();
}

// Keep the prompt overlay just below the sticky header (whose height changes when it wraps on a phone).
function fitHeader() {
  const h = document.querySelector("header").getBoundingClientRect().bottom;
  document.documentElement.style.setProperty("--hdr", Math.max(0, Math.round(h)) + "px");
}
addEventListener("resize", fitHeader);
addEventListener("scroll", fitHeader, { passive: true });
fitHeader();

// ---------------------------------------------------------------- polling with back-off
let pollDelay = POLL_MS;
async function poll() {
  const ac = new AbortController();
  const to = setTimeout(() => ac.abort(), 2500);
  try {
    const r = await fetch("/api/status", { signal: ac.signal, cache: "no-store" });
    if (r.status === 401) { location.reload(); return; }
    if (!r.ok) throw new Error("HTTP " + r.status);
    render(await r.json());
    pollDelay = POLL_MS;
    $("linklost").classList.add("hidden");
    fitHeader();
  } catch (e) {
    pollDelay = Math.min(POLL_MAX_MS, pollDelay * 2);
    $("linklost").classList.remove("hidden");
    fitHeader();
    ["ln-base", "ln-dart", "ln-rover", "ln-dartcam"].forEach((id) => $(id).classList.remove("ok"));
  } finally {
    clearTimeout(to);
    setTimeout(poll, pollDelay);
  }
}
poll();
