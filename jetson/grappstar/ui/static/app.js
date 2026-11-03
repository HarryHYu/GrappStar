// GrappStar operator page. Vanilla JS, no dependencies, works offline.
// Talks to the rover only through /api/status, /api/action/<name>, /stream, /snapshot, /artifact.
// Field names follow App.status() in grappstar/app.py and Mission.status() in grappstar/mission.py.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 300;
const POLL_MAX_MS = 3000;
const REPEAT_MS = 150;     // hold-to-run repeat period
const JOG_MS = 300;        // each jog command runs this long: releasing stops within JOG_MS (deadman)
const DART_MS = 350;
const ESTOP_HOLD_MS = 1000;

// Default mission steps (used when status.mission.steps is not provided by the App).
const DEFAULT_STEPS = [   // used only until the first status arrives (status.mission.steps is the truth)
  "PREFLIGHT", "LAUNCH_MODE", "AIM", "COCK", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "BASE_NEUTRAL", "WRAP",
  "ANCHOR", "TIDY", "FLY_IN", "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "FINAL_APPROACH", "LAUNCH_READY", "DONE",
];
// [key, label]; "seated" comes from the App (telemetry sw letter S)
const BASE_FIELDS = [["state", "State"], ["mode", "Mode"], ["drum_pos", "Drum"], ["pitch_deg", "Pitch deg"],
  ["cock", "Cock"], ["wraps", "Wraps"], ["line_m", "Line out m"], ["tension_n", "Tension N"], ["amps", "Current A"],
  ["vbat", "Battery V"], ["seated", "Dart seated"], ["fault", "Fault"]];
const DART_FIELDS = [["phase", "Phase"], ["seq", "Sequence"], ["rail_V", "Battery V"], ["current_mA", "Current mA"],
  ["armed", "Armed"], ["landed", "Landed"], ["event", "Last event"]];
const DART_KEYS = [["1", "deploy down"], ["!", "deploy up"], ["2", "kick out"], ["@", "kick back"],
  ["3", "drills in"], ["#", "drills out"], ["4", "foot in"], ["$", "foot out"]];
// Choices that never need the "Are you sure" dialog, even in a danger prompt.
const SAFE_CHOICE = /^(cancel|abort|no|back|stop|wait|hold|safe|retrieve|release and reshoot)/i;
// Which picture helps the operator decide each prompt: media shown inside the card, or a video scrolled into view.
const PROMPT_VIEW = { AIM: "rover", FIRE: "rover", TOW: "rover", FLIGHT: "dart", ANCHOR: "dart", CHECK: "dart",
  RELEASE: "dart" };

// Per-process session token the server writes into index.html; sent on every POST (review r2_H #8).
const TOKEN = (document.querySelector('meta[name="grappstar-token"]') || {}).content || "";

let status = null;
let pendingTap = null;          // {u, v} of a tap not yet confirmed by status.target

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
function num(v, d) { return typeof v === "number" && isFinite(v) ? v.toFixed(d) : "-"; }
function words(s) { return String(s || "").replace(/_/g, " "); }
function pick(obj, key) {
  if (!obj || typeof obj !== "object") return undefined;
  if (key in obj) return obj[key];
  for (const sub of ["sw", "telemetry", "tlm", "status", "last", "data"]) {
    if (obj[sub] && typeof obj[sub] === "object" && key in obj[sub]) return obj[sub][key];
  }
  return undefined;
}
function linkOk(x) {
  // Accepts a bool, or an object with connected / ok / up / alive, or a nested link field.
  if (x === null || x === undefined) return null;
  if (typeof x === "boolean") return x;
  if (typeof x !== "object") return !!x;
  if ("link" in x) return linkOk(x.link);
  for (const k of ["connected", "ok", "up", "alive", "online"]) if (k in x) return !!x[k];
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
$("toast").addEventListener("click", () => $("toast").classList.add("hidden"));

// ---------------------------------------------------------------- actions
async function action(name, args, quiet) {
  try {
    const r = await fetch("/api/action/" + encodeURIComponent(name), {
      method: "POST", headers: { "Content-Type": "application/json", "X-GrappStar-Token": TOKEN },
      body: JSON.stringify(args || {}), credentials: "same-origin",
    });
    let j;
    try { j = await r.json(); } catch (e) { j = { ok: false, msg: "HTTP " + r.status }; }
    if (r.status === 401) { location.reload(); return j; }
    if (r.status === 403 && j.reload) {      // server restarted: this page holds an old session token
      toast((j.msg || "page session expired") + ". Reloading...", true);
      setTimeout(() => location.reload(), 1500);
      return j;
    }
    if (!j.ok) toast(words(name) + ": " + (j.msg || "failed"), true);
    else if (!quiet && j.msg) toast(j.msg, false);
    return j;
  } catch (e) {
    toast(words(name) + ": no link to rover", true);
    return { ok: false, msg: "network" };
  }
}

// ---------------------------------------------------------------- STOP / ESTOP
(function setupStop() {
  const b = $("btn-stop");
  let timer = 0;
  b.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    action("stop", {});             // stop immediately on touch, not on release
    stopHolds();
    b.classList.add("armed");
    timer = setTimeout(() => { action("estop", {}); toast("ESTOP sent", true); }, ESTOP_HOLD_MS);
  });
  const end = () => { clearTimeout(timer); b.classList.remove("armed"); };
  b.addEventListener("pointerup", end);
  b.addEventListener("pointerleave", end);
  b.addEventListener("pointercancel", end);
  b.addEventListener("contextmenu", (e) => e.preventDefault());
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
    action(b.getAttribute("data-action"), args, b.hasAttribute("data-quiet"));
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

// ---------------------------------------------------------------- settings and manual range
$("btn-target-range").addEventListener("click", async () => {
  const r = parseFloat($("in-tr-range").value), b = parseFloat($("in-tr-bearing").value || "0");
  if (!isFinite(r) || r <= 0) return toast("Enter the target range in m", true);
  if (!isFinite(b) || Math.abs(b) > 180) return toast("Bearing must be -180 to 180 deg", true);
  pendingTap = null;
  await action("target_range", { range_m: r, bearing_deg: b });
});
document.querySelectorAll("input[data-key]").forEach((inp) => {
  inp.addEventListener("input", () => { inp.dataset.dirty = "1"; });
  inp.addEventListener("keydown", (e) => {
    if (e.key === "Enter") document.querySelector('button[data-set="' + inp.dataset.key + '"]').click();
  });
});
$("in-tr-range").addEventListener("keydown", (e) => { if (e.key === "Enter") $("btn-target-range").click(); });
document.querySelectorAll("button[data-set]").forEach((b) => {
  b.addEventListener("click", async () => {
    const key = b.getAttribute("data-set");
    const inp = document.querySelector('input[data-key="' + key + '"]');
    const v = parseFloat(inp.value);
    if (!isFinite(v) || v <= 0) return toast("Enter a number for " + words(key), true);
    const lo = parseFloat(inp.min), hi = parseFloat(inp.max);
    if ((isFinite(lo) && v < lo) || (isFinite(hi) && v > hi)) {
      return toast(words(key) + " must be " + inp.min + " to " + inp.max, true);
    }
    const r = await action("set", { key: key, val: v });
    if (r.ok) delete inp.dataset.dirty;
  });
});
function renderConfig(cfg) {
  cfg = cfg || {};
  for (const inp of document.querySelectorAll("input[data-key]")) {
    const v = cfg[inp.dataset.key];
    if (typeof v === "number" && !inp.dataset.dirty && document.activeElement !== inp) inp.value = String(v);
  }
  $("cfg-info").textContent = "Now: tow limit " + fmt(cfg.tow_limit_n) + " N, drag " + fmt(cfg.drag_n) +
    " N. Ballistics table " + fmt(cfg.ballistics) + ".";
  const warn = [];
  if (cfg.placeholder) warn.push("Ballistics table is a PLACEHOLDER, not measured on this launcher.");
  if (cfg.cam_uncalibrated) warn.push("Rover camera NOT calibrated: ranges from the video are rough guesses.");
  $("cfg-warn").textContent = warn.join(" ");
  $("cfg-warn").classList.toggle("hidden", !warn.length);
}

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
  // STOP: Escape from anywhere (even with focus in a field), Space when not typing (a typed space stays a space).
  if (e.key === "Escape" || e.key === "Esc" || (e.key === " " && !typing(e))) {
    e.preventDefault();
    if (!e.repeat) { stopHolds(); action("stop", {}); }
    return;
  }
  if (typing(e)) return;
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
  let n = 0, timer = 0;
  const load = () => { timer = 0; img.src = "/stream/" + cam + ".mjpg?r=" + (n++); };
  img.addEventListener("error", () => { if (!timer) timer = setTimeout(load, 2000); });
  img.addEventListener("load", () => {
    // fit the box to the real picture so there are no letterbox bars (and taps land where they look)
    if (img.naturalWidth && img.naturalHeight) {
      const ar = img.naturalWidth + " / " + img.naturalHeight;
      if (img.parentElement.style.aspectRatio !== ar) img.parentElement.style.aspectRatio = ar;
    }
  });
  img.restart = load;
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
$("cam-rover").addEventListener("click", async (e) => {
  const img = e.currentTarget, r = imageRect(img), box = img.getBoundingClientRect();
  const u = (e.clientX - box.left - r.x) / r.w, v = (e.clientY - box.top - r.y) / r.h;
  if (!(u >= 0 && u <= 1 && v >= 0 && v <= 1)) return;    // tap on the letterbox bars
  const tap = { u: u, v: v };
  pendingTap = tap;
  drawCrosshair();
  await action("target_click", { cam: "rover", u: +u.toFixed(4), v: +v.toFixed(4) }, true);
  if (pendingTap === tap) pendingTap = null;     // status.target now holds the answer
  drawCrosshair();
});
function drawCrosshair() {
  const t = status && status.target;
  let p = pendingTap;
  if (!p && t && typeof t.u === "number" && typeof t.v === "number" && (!t.cam || t.cam === "rover")) p = t;
  const x = $("xhair");
  if (!p) { x.classList.add("hidden"); return; }
  const img = $("cam-rover"), r = imageRect(img);
  x.style.left = (img.offsetLeft + r.x + p.u * r.w) + "px";
  x.style.top = (img.offsetTop + r.y + p.v * r.h) + "px";
  x.classList.toggle("hidden", !img.naturalWidth);
}
addEventListener("resize", drawCrosshair);
$("cam-rover").addEventListener("load", drawCrosshair);

function renderTarget(t) {
  const box = $("target");
  box.textContent = "";
  if (!t) {
    box.textContent = "No target. Tap the rover view, use Auto target or enter a range.";
    return;
  }
  if (typeof t.range_m !== "number") {
    box.appendChild(el("div", { class: "no" }, "No target: " + (t.msg || "pick another point")));
    return;
  }
  const src = t.source === "manual" ? "Typed range" : t.source === "click" ? "Tapped on the video" :
    t.source ? String(t.source) : "";
  if (src) box.appendChild(el("div", { class: "src" }, src));
  const b = typeof t.bearing_deg === "number" ? t.bearing_deg : 0;
  let rng = "Range " + t.range_m.toFixed(1) + " m";
  if (typeof t.range_lo_m === "number" && typeof t.range_hi_m === "number") {
    rng += " (" + t.range_lo_m.toFixed(1) + " to " + t.range_hi_m.toFixed(1) + ")";
  }
  box.appendChild(el("div", {}, rng + ", bearing " + (b >= 0 ? "+" : "") + b.toFixed(1) + " deg"));
  const turn = Math.abs(b) < 1 ? "Facing target (within 1 deg)" :
    "Turn rover " + Math.abs(b).toFixed(0) + " deg " + (b > 0 ? "RIGHT" : "LEFT");
  box.appendChild(el("div", { class: "turn" }, turn));
  const s = t.solution;
  if (s) {
    const reach = s.reachable;
    box.appendChild(el("div", { class: reach ? "yes" : "no" },
      (reach ? "Reachable" : "NOT reachable") +
      (typeof s.pitch_deg === "number" ? ", pitch " + s.pitch_deg.toFixed(1) + " deg" : "") +
      (typeof s.line_out_m === "number" ? ", line " + s.line_out_m.toFixed(1) + " m" : "") +
      (s.note ? " (" + s.note + ")" : "")));
  }
  if (t.uncalibrated) box.appendChild(el("div", { class: "src" }, "Camera not calibrated: range is a rough guess"));
}

// ---------------------------------------------------------------- prompt card
let shownPrompt = null;    // key of the prompt on screen
let answered = null;       // id answered, waiting for the mission to move on
function fitPrompt() {
  const sec = $("prompt");
  const h = sec.classList.contains("hidden") ? 0 : sec.getBoundingClientRect().height;
  // on a phone the card spans the width: leave room under the page so nothing is stuck behind it
  const docked = sec.getBoundingClientRect().width > innerWidth * 0.8;
  document.documentElement.style.setProperty("--promptH", (docked ? Math.ceil(h) + 8 : 0) + "px");
}
function promptMedia(p) {
  // Pictures shown inside the card: the landing zone verdict needs the report and the mosaic.
  const files = (status && status.results && status.results.files) || [];
  const tabs = [];
  if (p.step === "LANDING_ZONE") {
    if (files.includes("landing.png")) tabs.push(["Report", "/artifact/landing.png", true]);
    if (files.includes("mosaic.png")) tabs.push(["Mosaic", "/artifact/mosaic.png", true]);
    tabs.push(["Dart camera", "/stream/dart.mjpg", false]);
  }
  return tabs;
}
function showTab(tabs, i) {
  const img = $("prompt-img"), link = $("prompt-imglink");
  const [, url, still] = tabs[i];
  img.src = url + "?v=" + Date.now();
  link.href = still ? url : "#";
  link.onclick = still ? null : (e) => e.preventDefault();
  $("prompt-tabs").querySelectorAll("button").forEach((b, j) => b.setAttribute("aria-selected", String(j === i)));
}
function renderPrompt(p) {
  const sec = $("prompt");
  if (!p) {
    if (!sec.classList.contains("hidden")) {
      sec.classList.add("hidden");
      $("prompt-img").removeAttribute("src");     // closes a dart stream opened by the card
      fitPrompt();
    }
    shownPrompt = null;
    return;
  }
  const key = p.id + "|" + p.text + "|" + (p.choices || []).join(",");
  if (shownPrompt === key) return;
  shownPrompt = key;
  answered = null;
  sec.classList.toggle("danger", !!p.danger);
  sec.classList.remove("min");
  $("prompt-min").textContent = "Hide";
  $("prompt-step").textContent = (p.danger ? "Danger: " : "Confirm: ") + words(p.step || "operator");
  $("prompt-text").textContent = p.text || "";
  const tabs = promptMedia(p);
  sec.classList.toggle("media", tabs.length > 0);
  $("prompt-media").classList.toggle("hidden", !tabs.length);
  const tb = $("prompt-tabs");
  tb.textContent = "";
  tabs.forEach(([label], i) => {
    const b = el("button", { role: "tab", type: "button" }, label);
    b.addEventListener("click", () => showTab(tabs, i));
    tb.appendChild(b);
  });
  if (tabs.length) showTab(tabs, 0);
  else $("prompt-img").removeAttribute("src");
  const box = $("prompt-choices");
  box.textContent = "";
  for (const choice of p.choices || []) {
    const dangerous = !!p.danger && !SAFE_CHOICE.test(choice);
    const b = el("button", { class: dangerous ? "danger-choice" : "", type: "button" }, choice);
    b.addEventListener("click", async () => {
      if (answered === p.id) return;
      if (dangerous && !confirm("Are you sure: " + choice + "?\n\n" + (p.text || ""))) return;
      answered = p.id;
      box.querySelectorAll("button").forEach((x) => (x.disabled = true));
      // the text sent back is the text shown: the mission refuses it if the open prompt reads differently
      const r = await action("answer", { prompt_id: p.id, choice: choice, text: p.text || "" }, true);
      if (!r.ok) { answered = null; box.querySelectorAll("button").forEach((x) => (x.disabled = false)); }
    });
    box.appendChild(b);
  }
  sec.classList.remove("hidden");
  fitPrompt();
  // bring the video the operator needs for this decision into view above the card
  const view = PROMPT_VIEW[p.step];
  const target = view === "rover" ? $("vbox-rover") : view === "dart" ? $("vbox-dart") : null;
  if (target) {
    const r = target.getBoundingClientRect(), hdr = document.querySelector("header").offsetHeight;
    const cardTop = sec.getBoundingClientRect().width > innerWidth * 0.8 ? sec.getBoundingClientRect().top : innerHeight;
    if (r.top < hdr || r.bottom > cardTop) {
      const title = target.parentElement.querySelector("h2");
      let top = r.top + scrollY - hdr - 6;
      // keep the title too when it fits
      if (title && r.height + title.offsetHeight + 10 < cardTop - hdr) top -= title.offsetHeight + 4;
      scrollTo(0, Math.max(0, top));
    }
  }
  try { navigator.vibrate && navigator.vibrate(p.danger ? [200, 100, 200] : 120); } catch (e) { /* ignore */ }
}
$("prompt-min").addEventListener("click", () => {
  const sec = $("prompt"), min = !sec.classList.contains("min");
  sec.classList.toggle("min", min);
  $("prompt-min").textContent = min ? "Show" : "Hide";
  $("prompt-min").setAttribute("aria-expanded", String(!min));
  fitPrompt();
});
$("prompt-img").addEventListener("load", fitPrompt);

// ---------------------------------------------------------------- render status
function setPill(id, ok, label, title) {
  const p = $(id);
  p.classList.toggle("ok", ok === true);
  p.classList.toggle("bad", ok === false);
  p.textContent = (ok === true ? "✓ " : ok === false ? "✗ " : "? ") + label;
  p.title = label + (ok === true ? " OK" : ok === false ? " DOWN" : " unknown") + (title ? ": " + title : "");
}
function fillTable(id, src, fields) {
  const tb = $(id).tBodies[0];
  if (tb.rows.length !== fields.length) {
    tb.textContent = "";
    for (const [, label] of fields) {
      const tr = el("tr"); tr.appendChild(el("td", {}, label)); tr.appendChild(el("td", {}, "-")); tb.appendChild(tr);
    }
  }
  fields.forEach(([f], i) => {
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
  $("m-step").textContent = m.step ? words(m.step) : "-";
  const extra = [];
  if (m.reshoots) extra.push("reshoots " + m.reshoots);
  if (m.shot && typeof m.shot.line_out_m === "number") extra.push("line out " + m.shot.line_out_m.toFixed(1) + " m");
  $("m-extra").textContent = extra.join(", ");
  const err = m.error || m.last_error;
  $("m-error").textContent = err ? (m.state === "RUNNING" ? "Last error: " : "Error: ") + fmt(err) : "";
  $("m-error").classList.toggle("hidden", !err);
  const steps = Array.isArray(m.steps) && m.steps.length ? m.steps.map(stepName) : DEFAULT_STEPS;
  const done = new Set((m.history || []).map(stepName).map((s) => String(s).toUpperCase()));
  // current step: mission.step (mission.state is IDLE / RUNNING / DONE / ABORTED / FAILED)
  const cur = String(m.step || "").toUpperCase();
  const ol = $("m-steps");
  if (ol.dataset.k !== steps.join(",")) {
    ol.textContent = "";
    ol.dataset.k = steps.join(",");
    for (const s of steps) ol.appendChild(el("li", {}, words(s).toLowerCase()));
  }
  steps.forEach((s, i) => {
    const S = String(s).toUpperCase();
    const li = ol.children[i];
    li.classList.toggle("cur", S === cur && (m.state === "RUNNING" || S === "DONE"));
    li.classList.toggle("wait", S === cur && !!m.prompt);
    li.classList.toggle("done", (S !== cur || m.state === "DONE") && done.has(S));
  });
  const running = m.state === "RUNNING";
  // the dart is out after an abort or failure: offer the recovery right in the mission panel
  const base = (status && status.base) || {};
  const wraps = pick(base, "wraps");
  const dartOut = pick(base, "seated") === false || (typeof wraps === "number" && wraps > 0);
  $("m-recover").classList.toggle("hidden", !((m.state === "ABORTED" || m.state === "FAILED") && dartOut));
  document.querySelectorAll('button[data-action="start"], button[data-action="preflight"]').forEach((b) => (b.disabled = running));
  renderPrompt(m.prompt || null);
}
function loadReport(img, link, url, key) {
  img.onload = () => link.classList.remove("hidden");
  img.onerror = () => link.classList.add("hidden");
  img.src = url + "?v=" + encodeURIComponent(key);
  link.href = url;
}
let lzKey = null, moKey = null, flKey = null, imgGen = 0;
function renderResults(res) {
  res = res || {};
  const files = res.files || [];
  const tr = res.terrain;
  const k1 = tr ? JSON.stringify(tr) + files.includes("landing.png") : null;
  if (k1 !== lzKey) {
    lzKey = k1; imgGen++;
    const v = $("lz-verdict");
    v.className = "verdict";
    $("lz-reasons").textContent = "";
    if (!tr) { v.textContent = "No report yet"; $("lz-info").textContent = ""; $("lz-link").classList.add("hidden"); }
    else {
      v.textContent = words(tr.verdict || "?");
      if (tr.verdict) v.classList.add(tr.verdict);
      $("lz-info").textContent = words(tr.cls) + ", score " + fmt(tr.score) + "/100" +
        (typeof tr.slope_deg === "number" ? ", slope " + tr.slope_deg.toFixed(0) + " deg" : "") +
        (typeof tr.confidence === "number" ? ", confidence " + Math.round(tr.confidence * 100) + " percent" : "");
      for (const r of tr.reasons || []) $("lz-reasons").appendChild(el("li", {}, String(r)));
      if (files.includes("landing.png")) loadReport($("lz-img"), $("lz-link"), "/artifact/landing.png", imgGen + "_" + Date.now());
      else $("lz-link").classList.add("hidden");
    }
  }
  const mo = res.mosaic;
  const k2 = mo ? JSON.stringify(mo) + files.includes("mosaic.png") : null;
  if (k2 !== moKey) {
    moKey = k2; imgGen++;
    if (!mo) { $("mo-info").textContent = "No mosaic yet"; $("mo-link").classList.add("hidden"); }
    else {
      const used = typeof mo.n_used === "number" ? mo.n_used : Array.isArray(mo.used) ? mo.used.length : mo.used;
      $("mo-info").textContent = "Frames used " + fmt(used) + (Array.isArray(mo.notes) && mo.notes.length ? ". " + mo.notes.join(". ") : "");
      if (files.includes("mosaic.png")) loadReport($("mo-img"), $("mo-link"), "/artifact/mosaic.png", imgGen + "_" + Date.now());
      else $("mo-link").classList.add("hidden");
    }
  }
  const fl = res.flight;
  const k3 = JSON.stringify([fl && fl.t_launch, fl && fl.range_m, res.errors]);
  if (k3 !== flKey) {
    flKey = k3;
    $("fl-info").textContent = !fl ? "No flight yet" :
      "Range " + num(fl.range_m, 1) + " m, flight " + num(fl.flight_time_s, 2) + " s, launch " +
      num(fl.v0, 1) + " m/s at " + num(fl.launch_pitch_deg, 1) + " deg" + (fl.source ? " (" + fl.source + ")" : "");
    const ul = $("res-errors");
    ul.textContent = "";
    for (const e of res.errors || []) ul.appendChild(el("li", {}, String(e)));
  }
  // file links: the mission log is always there, the rest when the run produced them
  const fk = files.join(",");
  const box = $("files");
  if (box.dataset.k !== fk) {
    box.dataset.k = fk;
    box.querySelectorAll("a.art").forEach((a) => a.remove());
    for (const f of files) box.appendChild(el("a", { class: "btnlink art", href: "/artifact/" + encodeURIComponent(f), target: "_blank", rel: "noopener" }, f));
  }
}
let logKey = "";
function renderLog(lines) {
  lines = Array.isArray(lines) ? lines : [];
  const k = lines.join("\n");
  if (k === logKey) return;
  logKey = k;
  const ol = $("log");
  const atBottom = ol.scrollTop + ol.clientHeight >= ol.scrollHeight - 4;
  ol.textContent = "";
  for (const s of lines) {
    const t = typeof s === "string" ? s : JSON.stringify(s);
    ol.appendChild(el("li", { class: /\b(ERROR|FAIL)/.test(t) ? "e" : /\bWARN/.test(t) ? "w" : "" }, t));
  }
  if (atBottom) ol.scrollTop = ol.scrollHeight;
}
function camTitle(c) {
  if (!c) return "no camera";
  return (c.fps ? c.fps + " fps" : "no frames") + (c.recording ? ", recording" : "") + (c.last_error ? ", " + c.last_error : "");
}
function render(st) {
  status = st;
  const cams = st.cams || {};
  setPill("ln-base", linkOk(st.base), "BASE", st.base && st.base.link && st.base.link.last_error);
  setPill("ln-dart", linkOk(st.dart), "DART", st.dart && st.dart.link && st.dart.link.last_error);
  setPill("ln-rover", cams.rover ? linkOk(cams.rover) : false, "ROVER CAM", camTitle(cams.rover));
  setPill("ln-dartcam", cams.dart ? linkOk(cams.dart) : false, "DART CAM", camTitle(cams.dart));
  fillTable("t-base", st.base, BASE_FIELDS);
  fillTable("t-dart", st.dart, DART_FIELDS);
  renderResults(st.results);       // before the prompt: the landing zone card lists results.files
  renderMission(st.mission);
  renderTarget(st.target);
  drawCrosshair();
  renderConfig(st.config);
  renderLog(st.log);
  const sim = st.sim ? "SIMULATOR, " : "";
  if (typeof st.t === "number") $("clock").textContent = sim + "rover time " + new Date(st.t * 1000).toLocaleTimeString();
}

// Keep the link lost banner just below the sticky header (whose height changes when it wraps on a phone).
function fitHeader() {
  const h = document.querySelector("header").offsetHeight;
  document.documentElement.style.setProperty("--hdr", h + "px");
}
addEventListener("resize", () => { fitHeader(); fitPrompt(); });
fitHeader();

// ---------------------------------------------------------------- polling with back-off
let pollDelay = POLL_MS, wasLost = false;
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
    if (wasLost) {
      wasLost = false;
      document.querySelectorAll("img[data-cam]").forEach((img) => img.restart && img.restart());
    }
  } catch (e) {
    pollDelay = Math.min(POLL_MAX_MS, pollDelay * 2);
    $("linklost").classList.remove("hidden");
    if (!wasLost) {
      wasLost = true;
      stopHolds();
      for (const [id, label] of [["ln-base", "BASE"], ["ln-dart", "DART"], ["ln-rover", "ROVER CAM"], ["ln-dartcam", "DART CAM"]]) {
        setPill(id, null, label, "no link to the rover");
      }
    }
  } finally {
    clearTimeout(to);
    setTimeout(poll, pollDelay);
  }
}
poll();
