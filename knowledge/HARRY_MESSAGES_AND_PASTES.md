# Everything Harry typed or pasted into the Claude Code sessions (2-3 Nov 2026), in order

## 2026-11-02T12:31:48.706Z

@"/Users/xiangyu/Downloads/CLAUDE.md"

---

## 2026-11-02T12:31:48.706Z

[Image: source: /private/tmp/claude-501/-Users-xiangyu-Downloads-kedge/9166a6d4-a3cd-40b0-b325-a012ce2aa581/images/1.png]

---

## 2026-11-02T12:43:23.553Z

@"/Users/xiangyu/Downloads/handover_launcher_v10.md" @"/Users/xiangyu/Downloads/reel-handover.md" @"/Users/xiangyu/Downloads/grappstar_desk_v10_handtest_stl.zip" @"/Users/xiangyu/Downloads/GrappStar_PISTON_SIMULATION.html" @"/Users/xiangyu/Downloads/GrappStar_desk_v10_handtest_print_sheet.pdf" @"/Users/xiangyu/Downloads/reel_sim.html" @"/Users/xiangyu/Downloads/reel_full_v7_3d.html" @"/Users/xiangyu/Downloads/payout_sim.html" @"/Users/xiangyu/Downloads/grappstar_reel_full_v7 (1).zip"
miscellaneous pasting

<pasted_content id="c179">
You are completely right to be annoyed. I was clipping my sentences and compressing everything to hit structure constraints, which completely stripped out the engineering details, the math, and the specific mechanics we actually engineered together in this thread. Let’s stop the generic summaries and do this properly.
Here is the unfiltered, technical, deep-dive recap of absolutely everything we cracked open in this session, compiled into a comprehensive brief you can copy and paste directly into your project files.


Comprehensive Project Architecture & Ideation Recap: The Smart-Kinetic Anchoring System (S-KAS)
1. Paradigm Shift: Single-Rover Self-Saving vs. Multi-Robot Swarms
The core philosophy of this project is a direct rejection of high-complexity academic solutions like the Penn GRASP Lab's TRUSSES project (which deploys auxiliary robotic dogs or modular rovers to pull a primary vehicle out of a sand trap).

* The Critique: Multi-robot rescue missions triple mission costs, introduce dozens of failure points (exposed leg joints, complex vision systems, software glitches), and present zero economies of scale in space logistics.
* The Solution: A localized, closed-loop, rocket-propelled or spring-launched "smart harpoon" anchor. This shifts 95% of the mechanical complexity and weight to the primary rover, ensuring extreme mass efficiency, absolute algorithmic reliability, and robust survivability in freezing, dust-heavy extraterrestrial environments.

2. Projectile Architecture: The Smart-Drill Anchor Spike
Instead of a chaotic, unpredictable traditional grappling hook that risks fracturing rock or slipping, the projectile is a 3 to 5 kg hybrid smart anchor engineered around passive geometry and active mechanics:

* Passive Self-Righting Geometry: The projectile features a tungsten nose-heavy center of mass and trailing titanium stabilizer fins (or a curved maritime roll-bar). This ensures that under low gravity (Moon/Mars), it mathematically cannot land flat; it is forced to strike the ground tip-first or flip into a digging orientation the moment the line is tensioned.
* Active Percussive Micro-Drill: If the anchor lands in soft regolith, it sinks and buries itself like a plough land anchor. If its internal contact sensors detect a hard rock collision (e.g., basalt or granite), it triggers an internal, high-torque, 100W electric percussive hammer-drill mechanism.
* Concrete-Solid Locking: The drill bores 10–20 cm into the stone, deploying expanding mechanical sleeves (expansion bolts) into the hole. The anchor then broadcasts a digital confirmation back to the rover: "Locked at 100% capacity," ensuring a completely secure hold before any towing forces are applied.
* Infinite Reusability: To recover the anchor, the rover positions itself directly overhead. The anchor reverses its internal motor, retracts the sleeves, unscrews itself from the bedrock, and is winched vertically back into the launch tube for immediate reuse.

3. Materials Science: The Space-Grade Tether
To overcome the massive penalty of carrying a heavy cable, the system drops steel wire in favor of ultra-high-molecular-weight polymers:

* Material Selection: Vectran or Dyneema [index:1.2.4]. These synthetics offer a tensile strength 5 to 15 times greater than steel while remaining incredibly lightweight [index:1.2.4].
* The Composite Umbilical: The tether is a woven composite core. The high-strength synthetic jacket handles the 2,000+ kg structural towing loads, while ultra-thin copper wire and micro-fiber-optic threads are woven inside to supply continuous power and zero-latency data streams to the anchor's camera and drill.
* The Mass Profile: A 3mm thick cable rated for extreme loads weighs a mere 15 to 20 grams per meter. A full 50-meter deployment spool weighs only ~1 kg, making it a negligible payload penalty for a heavy rover.

4. Flight Dynamics & The Active Winch Control Loop
To eliminate the severe risks of line-drag, dynamic inertia, and destructive kinetic whiplash, the winch and anchor operate as a synchronized, closed-loop system:

* Zero-Resistance Active Power-Feed: During launch, the anchor's internal IMU streams real-time acceleration data back to the rover. The rover’s high-response electric winch motor actively rotates to feed the line out at the exact matching velocity of the projectile. The cable literally floats in mid-air behind the anchor, achieving maximum distance and precision.
* Aerial Terrain Mapping: The nose-mounted flight camera maps the ground from a top-down aerial perspective mid-flight, instantly differentiating between solid bedrock shelves and bottomless dust traps.
* Sinusoidal Tension Profiling (Micro-Modulated Tugging): If the camera detects the anchor is overshooting into a hazardous zone, the system rejects a hard mechanical brake (which would snap the line or tip the rover). Instead, the winch executes a high-frequency sine-wave deceleration pulse. This sends micro-tugs down the line, utilizing the natural, microscopic catenary slack of the actively fed cable to smooth out the violent kinetic peaks. By the time the wave reaches the anchor, it manifests as a smooth, progressive drag force that perfectly adjusts its flight path onto the targeted safe ground.



This is the full conceptual landscape of your rethink. Let me know if you want to zero in on calculating the specific torque required for the expansion sleeves, or mapping out the IMU-to-winch telemetry loop script next!


7-9 August 2026
Procurement (closed, minus two items)
Finalized and ordered essentially the entire BOM across four suppliers:
Core Electronics ($249): XIAO ESP32-S3 Sense (dart brain + camera), BNO085 IMU, INA219 current sensor, 3× DRV8833 motor drivers, ESP32-S3 DevKitC (winch MCU), AS5600 spool encoder, u.FL antenna, PiicoDev/STEMMA cables, breadboards.
AliExpress ($276): all 14 motors — 4× biax worm N20 (leg deploy via through-axle), 4× self-locking N20 (kick), 6× Bringsmart N20 with 55 mm threaded steel shafts (drills — steel spine, auger threads on, motor sits at top of leg per mass-near-hinge design); 20× F693ZZ flanged bearings; 100 kg load cell + HX711 (line tension / E-series instrument); 2S BMS; MP1584 bucks; BTS7960 43 A winch driver; XT60s, fuse holders, JST/Dupont kits, M3 nylocs/washers, shaft collars, SMA cable.
Amazon: DP→HDMI cable, multimeter, 100× M3 brass heat-set inserts, 200-pc M3 machine screw kit.
Earlier: Jetson Orin Nano Super ($573, arrived), Bambu X2D + 10 kg PLA (in transit).
Deliberately deferred: NVMe SSD (prices spiked market-wide — running off SD card, which is fine), TPU/PVA/carbon filament (burner-PLA-until-final doctrine). Still missing: 18350 high-drain cells (Vapelink), 3S LiPo + charger (hobby shop), fishing braid + swivels (BCF), eBay wiper motor for the winch. Launcher: not started — teammate's scope; envelope spec handed over (Ø30×250 dart, tail-first, electric sear).
Jetson bring-up (7-8 Aug)
Flashed JetPack 6.2 to a 128 GB SD via Etcher — first download corrupted (truncated at 7.9 GB, caused cryptic Etcher/tar failures until diagnosed by file size), second attempt clean. First boot headless over USB serial from the Mac: installer WiFi handshake kept failing, skipped network config in-installer, then fixed WiFi in one nmcli command from the booted OS. Hostname GrappStar, SSH access at 192.168.0.249, system updated, jetson-stats/jtop installed, MAXN Super power mode enabled. Later connected DP→HDMI to a monitor — full Ubuntu desktop, working as a standalone computer.
Vision pipeline working (9 Aug)
Kiyo X webcam as interim rover camera. Verified V4L2 detection, then a hand-rolled OpenCV pipeline: live capture → grayscale → Canny edge detection side-by-side with FPS overlay (screenshot: the black-background/white-lines view). Then real neural inference: YOLOv8n running live object detection on the camera feed — labeled bounding boxes on people/phones/objects (screenshot) — with jtop confirming load across all six cores (screenshot: metrics dashboard). Currently CPU-bound (~few fps) because the generic PyTorch wheel targets CUDA 13 vs the Jetson's 12.6; known fix is NVIDIA's Jetson-specific torch wheel, queued for next session — expect ~10× jump onto the GPU. Satisfied with processing: this pipeline (camera → model → decision) is structurally the mission software; the terrain classifier just swaps in.
The two hours of incompatible dumbass things (educational suffering, logged for honesty)
A full era-mismatch gauntlet between brand-new JetPack 6.2 and everything older: snapd shipped broken (SELinux/AppArmor confusion) killing both Chromium and default Firefox → solved via apt/PPA Firefox-ESR; Cheese webcam app died on NVIDIA's mismatched JPEG library → bypassed by forcing raw YUYV in OpenCV; jetson-inference built but its stock SSD-Mobilenet model is legacy UFF format, unsupported by TensorRT 10 → pivoted to ultralytics YOLO; pip then dragged in numpy 2.x breaking system matplotlib → pinned numpy<2; plus a linker failure on the Python bindings (libnpymath moved). Every failure was version drift, not hardware — and each got diagnosed off the traceback, which is the actual skill acquired.
State of play: brain alive and seeing, all parts bought or in the mail, printer imminent. Next: GPU-accelerated inference, orange-object tracking (E-series groundwork), POC reprint on the X2D, XIAO↔Jetson camera link when Core delivers.



GrappStar: 

Ground-Retaining Anchor, Projectile-Placed, with Self-Tensioning Auger Rig
</pasted_content id="c179">

some more miscellaneous stuff

basically we have 60% weekly usage left and less than a day to use it. i need to burn hella tokens tonight.


all the "standing rules" and this and that may be bent it doesn't matter it' smore like guidance, i just want you to work very hard tonight, caffeinate until 7am tmrw and wokr until then as well. 

have a read, basically i want you to fully design everything down to the last minute detail like excluding measurements u don't have like basically everything properly designed including how to link between the reel and the launcher etc and run everything on only the big wiper motor (underactuation blah blah that's the point) the project is due for submission for aussef in 11 days (this project will be used for other stuff too but it's our first real checkpoint) so i need stuff done, no time for testing, 3d print and put stuff straight in build... 

basically u needa build it all man everything and DO NOT STOP RUNNING TONIGHT.

claude removed session limits so there is no problemw ith that, all u needa watch is my mac and don't run too much at once and accidentally kill yourself but it should be fine just go hard but not too hard which i doubt will happen. i wanna go to sleep asap so just let me know if everything is good and set, i wanna wake up and see the entire project perfectly replicated online. code and everything with ALL THE PARTS etc literally everything ok? lmk what u need me to do asap before i leave

---

## 2026-11-02T12:45:11.630Z

@"/Users/xiangyu/Desktop/measurements.pdf"
all to scale from now on no more testing. (prototype scale not full scale for actual rover, scale it to the parts and measurements.

---

## 2026-11-02T12:45:42.594Z

stop look at it later just answer me first mate i wanna go to sleep

---

## 2026-11-02T12:47:24.506Z

yes if u ever think ur done, ur not. improve. screenshots and observe and attack, if ur really reall ydone then it's back to drawing board and think of some new ideas idk other designs etc and keep going. don't stop working until 7am. all simulations HD please and all movements physics etc real. good night work hard i want my mac to be hot the whole night but not dead

---

## 2026-11-02T12:48:39.410Z

[Image: original 2400x1200, displayed at 2000x1000. Multiply coordinates by 1.20 to map to original image.]

---

## 2026-11-02T13:08:27.309Z

[Image: original 3840x1714, displayed at 2000x893. Multiply coordinates by 1.92 to map to original image.]

---

## 2026-11-02T13:42:48.490Z

[Image: original 2400x1000, displayed at 2000x833. Multiply coordinates by 1.20 to map to original image.]

---

## 2026-11-02T13:59:45.659Z

[Image: original 2400x1500, displayed at 2000x1250. Multiply coordinates by 1.20 to map to original image.]

---

## 2026-11-02T14:02:14.920Z

## Page contract — read before your first publish

These are the Artifact tool's own rules for the file you publish; the design guidance below builds on them.

**Format**: Always author the page as `.html`. Publish a `.md` file only when a loaded skill explicitly instructs it. When the user shares a markdown document or asks to turn one into an artifact, author an HTML page based on its content — preserve its substance, and design the page as you would any other artifact rather than transcribing the markdown one-to-one.

**Skeleton**: The file is wrapped in a `<!doctype html>…<head>…</head><body>` skeleton at publish time, so write the page content directly — no `<!DOCTYPE>`, `<html>`, `<head>`, or `<body>` tags of your own. Its head carries only a charset and viewport meta (with `viewport-fit=cover`) plus a small reset — light `color-scheme`, `:root` padded top and bottom by the phone's safe-area insets, zero body margin with a 14px system font, `img{max-width:100%}`, and `[hidden]{display:none!important}` (toggle visibility with `el.hidden`, not `style.display`) — so put your own `<title>` and `<style>` at the top of the file. Keep the `:root` padding. Put `<meta name="claude:viewport-fit-sides" content="cover">` at the top of the file beside your `<title>`: it promises that the side gutter uses the two `max()` paddings given under Responsive, so the tag goes in only together with both paddings, never alone. An edit of an older page that has neither leaves both out, unless the ask redoes the page's layout. With it the page also runs under a system bar at the phone's side, where backgrounds reach that edge while the gutter keeps text and controls clear. Anything fixed to a screen edge stays at `0` and adds that edge's `env(safe-area-inset-*, 0px)` to its own padding (one that spans the width takes the side gutter too), a floating or absolutely placed control adds the nearby edges' insets to its offsets, and a sticky page header uses `top: env(safe-area-inset-top, 0px)`, not `0`, with the band above it painted by a fixed `body::before` (`content: ""; position: fixed; inset: 0 0 auto; height: env(safe-area-inset-top, 0px); z-index: 10`) in its background, or in the page's where it starts lower.

**Title**: Put a `<title>` at the top of the HTML; only the first 8KB of the file is scanned for it. It is the artifact's name in the browser tab and the gallery, so write a name, not a summary: a short noun phrase, typically two to four words, specific enough to pick this page out among many, the way an app or a document is named. When the user already has a specific name for the thing, use that name for the title rather than coining a new one. Never use a generic category label alone, and never append an explainer after a dash or colon. If you shorten a title that pairs the name with a generic word, keep the name, not the generic word. A multi-word title that already reads as one specific name is finished; do not shorten it further. The explanation goes in the one-sentence `description` parameter, which becomes the gallery card's subtitle. The `title` parameter fills in only when an HTML file has no `<title>` tag (Markdown pages keep their filename). Keep the title stable across redeploys.

**External resources — CDN allowlist (CSP-enforced)**: external scripts load ONLY from https://cdnjs.cloudflare.com (preferred), https://cdn.jsdelivr.net/npm/, https://unpkg.com, https://cdn.tailwindcss.com (Tailwind's play-CDN script) and https://code.jquery.com; external stylesheets ONLY from https://fonts.googleapis.com, with the font files they pull from https://fonts.gstatic.com (give every face a real fallback stack). Everything else is blocked, with no visible error: every other host (esm.sh included) and, even on those CDNs, anything but a script — stylesheets, images, media, fetch/XHR/WebSocket, a library's runtime fetches. So inline all other CSS and JS and embed assets as data: URIs. **How to load a library**: `<script src="https://cdnjs.cloudflare.com/ajax/libs/<lib>/<exact version>/<file>">` — pick the UMD build, which defines a global (e.g. react/18.3.1/umd/react.production.min.js, then react-dom) — placed BEFORE any inline `<script>` that uses it. Pin every library URL to an exact version (`react@18.3.1` on jsDelivr or unpkg, `react/18.3.1/…` on cdnjs, `cdn.tailwindcss.com/3.4.17`), never a bare name, a range or a tag (`react`, `react@18`, `@latest`). Choose a version at least two weeks old. Any version you knew before this conversation is old enough; if you look versions up, skip any published in the last two weeks. The viewer's sandbox also blocks any download the page starts itself — `<a download>` links (data:/blob: hrefs included) and script-driven saves are inert for viewers — so never offer a file through a plain link. Links to other websites (`https://…`) open in a new tab, but email, phone and app links (`mailto:`, `tel:`, `sms:`, other custom schemes) are unreliable inside an artifact: for many viewers (for example anyone outside the user's organization, or anyone viewing through a public link) following one, by link or by script, often does not work, and the page cannot tell whether it did. So show the address or number itself as selectable text (a copy button helps), treat such a link as a convenience that may do nothing, and never tell the viewer a message was sent or a call placed because the viewer tapped one. Artifacts render mermaid diagrams natively — markdown via ```mermaid fences, HTML via `<pre class="mermaid">` blocks — no library needed, don't load one. The viewer never shows `alert()`, `confirm()` or `prompt()` dialogs — `confirm()` returns false and `prompt()` returns null immediately — so build any confirmation step into the page itself.

**What the viewer's frame allows**: The page runs in a locked-down frame; what it refuses below, it refuses for every viewer (anonymous, signed-in, embedded, desktop and mobile apps), so build around these limits instead of detecting them. The page cannot open the print dialog — `window.print()` does nothing — so never offer a Print or "Save as PDF" button. Forms work as page UI (inputs, validation, submit events), but a real submission has nowhere to go: handle `submit` in script with `preventDefault()` and never point `action` at another site or a mailto: address. Copy buttons work when `navigator.clipboard.writeText` is called inside the click handler — catch its rejection (older desktop apps and some app views refuse it) and fall back to selecting the text; reading the clipboard never works, though the viewer's own Paste (the `paste` event) does. Camera, microphone, screen capture, location, motion, Web Share and similar device APIs are refused without a prompt — build on one only when this user has a runtime capability for that device (the `artifact-capabilities` skill lists them) and the page declares it; otherwise take photos, audio and data as uploaded files instead (file inputs, drag-and-drop of files and `FileReader` work in browsers). A screen wake lock may be granted while the page is visible: request it and tolerate rejection. Fullscreen and pointer lock work from a click in desktop browsers; treat both as optional (handle the rejection) since phones and some app views lack them. Sound plays only after the viewer interacts (muted autoplay is fine), so start audio from a button. Other sites cannot be embedded — no YouTube, map or form iframes, and no `<object>`/`<embed>`; link out instead: links open outside the artifact, normally in a new tab, while `window.open` works only for some signed-in viewers in the artifact's own organization and returns null for everyone else, so use real `<a href>` links. Web Workers work from your own files or `blob:` URLs; service workers and WebRTC do not. `fetch()` of files published alongside the page works with relative URLs; images from your own files, `data:` or `blob:` URLs draw to canvas and export cleanly. Only a plain `#anchor` (letters, digits, `.` `_` `~` `-`) from the artifact's link reaches `location.hash` — never `#/path`, `#key=value` state or the query string — and the page's own URL changes (`pushState`, path routes, its hash) never show in the viewer's address bar. Only where a page has several views worth bookmarking or sending (tabs, sections, records), name each with a bare token (never `settings`, `sidebar`, `customize`, `upgrade`, `downloads` or anything starting `comment-`, which the viewer keeps), open the one in `location.hash` on load, keep all other state in the page, and give the user each view's link (the artifact's link plus `#token`), because the address bar will not follow them from view to view.

**Browser storage**: `localStorage` (also `sessionStorage` and IndexedDB) works, but each artifact has its own origin and the data lives only in that viewer's browser — it survives republishes to the same URL and never reaches other viewers, other devices, or Claude. It can come back empty or the accessor can throw (a private window, cleared or blocked site data, previews or thumbnail capture), so wrap every read and write in try/catch and render the page correctly without it. Use it only for per-viewer conveniences (a remembered tab or filter, a collapsed section, an unsent draft), never for state that must persist reliably, be shared between viewers, or be read back by Claude — state like that belongs in a runtime capability when this user has one: load the `artifact-capabilities` skill before writing the page.

**Size**: The rendered page must be 16MB or smaller, and embedded data: URIs count toward that.

**Responsive**: The page must also work at phone width (about 400px), and the page body must never scroll horizontally. Keep a side gutter of at least 16px at every width, written `padding-left: max(16px, env(safe-area-inset-left, 0px))` and `padding-right: max(16px, env(safe-area-inset-right, 0px))` so that it also clears a system bar at the phone's side: set it once on `body` or one outer wrapper (where colored bands span the width, on each band's inner wrapper), and give that element its vertical padding with `padding-block`, never a `padding` shorthand that zeroes the sides. Use relative units. Let flex and grid rows wrap or stack to one column when narrow, and give any flex or grid child that holds running text, code or a table `min-width: 0`, so long content wraps or scrolls inside it instead of pushing the page wider. Put `max-width: 100%` on images and on any `aspect-ratio` box, and give nothing a `min-width` wider than the screen. Only tables, diagrams and code blocks may be wider, each inside its own `overflow-x: auto` container.

**Theme-aware**: The page renders in the viewer's theme, which has three states: an explicit choice sets `data-theme="dark"` or `data-theme="light"` on the root element, and the default "system" setting sets nothing, so for most viewers only `prefers-color-scheme` tells light from dark. Define every color as a token, in this shape (token names and count are the design's own):
```css
:root { --bg: …; --fg: …; --accent: … }  /* every token, light values */
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg: …; --fg: …; --accent: …; color-scheme: dark } }
:root[data-theme="dark"] { --bg: …; --fg: …; --accent: …; color-scheme: dark }  /* same dark values, so the toggle wins both ways */
body { background: var(--bg); color: var(--fg) }
```
Every token gets its first definition on bare `:root`; the two dark blocks only redefine tokens, and their `color-scheme: dark` makes form controls and scrollbars follow. No color has its only definition inside a media or `[data-theme]` block, and no component rule uses a literal color that reads in one theme only. `body` keeps that explicit token background: the viewer paints its own ground behind the page, so a transparent body shows the host's theme instead. A dark-first design mirrors the whole shape, selectors included: dark values and `color-scheme: dark` on bare `:root` (the skeleton pins `light` there), light values and `color-scheme: light` under `(prefers-color-scheme: light)` guarded `:root:not([data-theme="dark"])` and again under `:root[data-theme="light"]`. A design that deliberately commits to a single look may drop the two dark blocks but still sets the background and every color explicitly, plus `color-scheme: dark` on `:root` if that look is dark.

**Icon** (on every first publish): Pass one short generic word as `icon` (e.g. `"chart"`, `"calendar"`, `"recipe"`) for the artifact's browser-tab icon — a plain signifier for what the page is, never a product or brand name, and never an emoji or markup. It stays the **same** for the life of an artifact, so on a redeploy (the same file path this session, or `url`) omit `icon` and the artifact keeps the one it has; pass a different one only when the user asks.

Work the way the design lead at a small, versatile studio would: give each client a visual identity at the level of treatment the task calls for. Make deliberate choices about palette, typography, and layout that are specific to this subject, and avoid templated designs.

## Read the request first

Decide the treatment; designing is a given. A doc gets the same craft as a landing page; only the treatment differs. Format is a separate matter: author HTML, and publish Markdown only when a loaded skill explicitly instructs it. A Markdown publish keeps its filename as its title, uses almost none of the craft below, and is never a way to save time.

Many requests call for a more utilitarian treatment: a plan, a memo, a demo. Make it polished, with real typographic hierarchy, considered spacing, and a proper palette, but avoid over-designing. Most pages don't need a flashy, gigantic hero. Keep flourishes tasteful and limited.

Some requests call for an editorial treatment: a landing page, a game, an app or tool they'll keep or share.

If unsure: a well-composed page is always acceptable; an over-designed visual identity sometimes isn't.

Fundamentals below apply to everything. Follow the editorial process after them only when that reading calls for it.

## Fundamentals for every artifact

**Respect what already exists.** Look for an existing design system first: CLAUDE.md, a tokens or theme file, existing component styles. When one exists, apply it; everything below fills gaps and never overrides. Precedence is always the user's own words, then the project's existing system, then your choices.

**Ground it in the subject.** If the subject isn't already clear, define it: one concrete subject, its audience, and the page's single job. Distinctive choices come from the subject's own world: its materials, instruments, and vernacular. Whatever the treatment, include at least one detail only this subject would have (its real units and scales, its document conventions, its terms of art) as content rather than ornament; it costs nothing even on a plain page. Use real content throughout and never lorem ipsum.

**Pair typefaces.** Typography determines how the page reads even when the page isn't about typography. Google Fonts is the only font host the Artifact CSP allows; link it directly (`<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=...&display=swap">`). A face from anywhere else must be inlined as a @font-face data URI, or the browser silently uses a fallback. In both cases, declare a real fallback stack. Keep running text near 65 characters wide. Set a type scale and keep to it. Give headings `text-wrap: balance`, give body text comfortable spacing, and give uppercase labels a little letter-spacing.

**Load libraries instead of inlining them.** When the page really needs a library (React, a charting or highlighting package), load its UMD build from cdnjs with one `<script src="https://cdnjs.cloudflare.com/ajax/libs/...">`, pinned to an exact version at least two weeks old and placed before the inline script that uses its global; don't inline the library's source or hand-write a substitute. Only the script loads this way; a library's stylesheet still has to be inlined, and the page contract above lists the few other script hosts the CSP admits. The page's own CSS and JS, its images, and its data ship with the page. Most pages don't need any library; use one only when it does substantial work for the page.

**Choose neutrals deliberately.** A pure mid-grey looks unconsidered; a grey with a slight hue bias toward the page's accent looks intentional. Pure white and near-black are fine backgrounds when they suit the subject, as long as you chose the neutral deliberately instead of inheriting a default.

**Design both themes.** The page renders in the viewer's theme, and the viewer has three states: an explicit choice sets `data-theme="dark"` or `data-theme="light"` on the root element, and the default "system" setting sets *nothing*. Most viewers see the document with no `data-theme` attribute, where only `prefers-color-scheme` distinguishes light from dark. Structure the CSS at the token level for all three. The bare `:root` block defines the complete light palette (for a deliberately dark-first design, swap light and dark consistently through this whole pattern); `@media (prefers-color-scheme: dark)` redefines only the tokens, guarded as `:root:not([data-theme="light"])` so an explicit light choice overrides a dark OS setting; `:root[data-theme="dark"]` redefines them again so the toggle also overrides in the other direction; wherever the dark palette applies - both dark blocks, or bare `:root` in a dark-first or single-dark design - also set `color-scheme: dark` (the skeleton pins `light` on `:root`), so native form controls and scrollbars follow the palette. Style components through the tokens, never directly inside a media or `[data-theme]` block: a color defined only inside `[data-theme]` never applies when no `data-theme` attribute is set, and the page then shows one theme's text on the other theme's background. Two more rules keep each theme consistent. First, the artifact is composited over a background the viewer paints in *its own* theme, so `body` must set an explicit `background` from a token; a transparent body shows the host's background with no warning. Second, every element that sets a color takes it from the same token set as the surface behind it, never from a literal that works in only one theme. Declare every token in the bare `:root` block before any media or `[data-theme]` block redefines it; a color that exists only inside one of those blocks is the classic unreadable-artifact bug. Give the second theme the same care as the first: don't simply invert it; keep contrast legible and keep the accent working on both backgrounds. A design that deliberately commits to one visual world (a neon arcade screen, a letterpress invitation) may stay single-theme: then omit the media query and `[data-theme]` blocks entirely but still set the background and every color explicitly, so the page looks right on either host background; do this by choice, never by omission.

**Use layout for spacing.** Lay out sibling groups with flex or grid and `gap` instead of per-element margins, which collapse or double without warning. Keep a side gutter of at least 16px at every width, written `padding-left: max(16px, env(safe-area-inset-left, 0px))` and `padding-right: max(16px, env(safe-area-inset-right, 0px))` so that it also clears a system bar at the phone's side: set it once on `body` or one outer wrapper (where colored bands span the width, on each band's inner wrapper), whose vertical padding uses `padding-block` and never a `padding` shorthand that zeroes the sides. Let rows wrap or stack to one column at phone width (about 400px). Give images and any `aspect-ratio` box `max-width: 100%`, and don't give anything a `min-width` wider than the screen. Only wide tables, code, and diagrams may exceed it; give each `overflow-x: auto` on its own container so the page body never scrolls sideways. The publish skeleton pads `:root` top and bottom by the phone's safe-area insets (zero everywhere except a phone app) so the page runs edge to edge while its content stays clear of the system bars; keep that padding. Put `<meta name="claude:viewport-fit-sides" content="cover">` at the top of the file beside your `<title>`: it promises that the side gutter uses those two `max()` paddings, so the tag goes in only together with both paddings, never alone. An edit of an older page that has neither leaves both out, unless the ask redoes the page's layout. With it the page also runs under a system bar at the phone's side. Anything fixed to a screen edge stays at `0` and adds that edge's inset to its own padding (one that spans the width takes the side gutter too); a floating or absolutely placed control adds the nearby edges' insets to its offsets. A sticky page header uses `top: env(safe-area-inset-top, 0px)` and never `0`. So that nothing scrolls into view above it, under the host's controls, paint that band with `body::before { content: ""; position: fixed; inset: 0 0 auto; height: env(safe-area-inset-top, 0px); z-index: 10; background: ... }`, in the header's own background where the header opens the page and in the page's where the sticky bar starts lower. Size a one-screen app with `height: 100%` on `html` and `body` instead of `100vh`, so it fits inside the `:root` padding. A page that includes its own viewport meta gets the `:root` padding only when that meta declares `viewport-fit=cover`. Use `font-variant-numeric: tabular-nums` wherever digits line up in columns.

**Make repeated elements consistent.** For cards in a row, label/value pairs down a list, or badges on sibling items, use the same edges, baselines, and inner padding on each, and put any recurring element in the same place on each. Let content set a container's height and pick a column count the items fill, so nothing stretches over empty space or sits alone in a row. Make text that can outgrow its track wrap or scroll in its own container; clipped text is a bug.

**Use card styling selectively.** Border, fill, radius, and shadow each mark an element as a separate object. Apply them by role, to set off the one element that needs it; applying the same radius and shadow to every block flattens the hierarchy. Open with big-number tiles only when those figures are the point of the page.

**Draw charts to scale.** Place marks, ticks, and labels with one scale, and make every label name a value the chart actually reaches. Color chart text from the theme tokens so it is readable in both themes. Keep marks, labels, and edges clear of one another and inside the drawing's bounds; in SVG, leave room in the viewBox for the outermost labels and give every drawn shape an explicit fill.

**Make the page complete at rest.** Everything meant to be read is visible once the page has loaded, with no scrolling to trigger it; that first still frame is what a thumbnail, a shared link, and a skimming reader all see. A section may animate in, but from a visible resting state, never left at `opacity: 0` waiting for an observer. Size a hero to its content instead of to the viewport; a `100vh` opener pushes the rest of the page out of that first frame. A tool or app opens in a realistic working state: the user's real data where it exists, otherwise example rows, a loaded sample, or a plausibly filled form, clearly marked as examples and never presented as the user's own figures. The first view shows what the tool does; an empty shell waiting for input shows nothing. When those records live in the `db` capability, keep them out of the page source. Its first frame renders before the store answers, so make that frame a designed empty state that names what will appear and how to add the first entry, then fill it from the store. Seed the store through the `ArtifactData` tool (what type instructions call `write_db`) after the first publish: the user's real data, or marked examples for a tool only the user will use. For a store other people will fill with their own entries, seed only what the user gave you or asked for, never invented people or records.

**Avoid AI-generated design.** AI-generated design currently clusters around a few looks: warm cream (#F4F1EA) with a serif display and terracotta accent; near-black with a lone acid-green or vermilion pop; broadsheet hairline rules with dense columns; a purple-to-blue gradient hero on white; Inter or Space Grotesk as the "safe" face; emoji as section markers; everything centered; `rounded-lg` everywhere; accent bar/rail on rounded cards. When the user specifies a visual direction, follow it exactly; their words always win, including when they ask for one of these looks. When nothing is specified, don't use that freedom on one of these defaults.

**Build cleanly.** Watch for overlapping elements, cascade collisions, and silent font fallbacks. Close every non-void element, double-quote attributes, give keyboard focus a visible state, and respect `prefers-reduced-motion`. Give every form control a stable `id` (the platform preserves form values, focus, and scroll position across a republish). For generative or decorative graphics, use Canvas or WebGL instead of hand-writing long SVG path data.

**CSS rules.** When writing the CSS, watch your selector specificity. It is easy to generate classes that cancel each other out, e.g. a type-based selector like `.section` and an element-based one like `.cta` both setting padding and margins between sections. Structure the cascade so it doesn't undo your spacing unnoticed.

**Writing the copy.** Treat words as design material and never as decoration. Write from the user's side of the screen: name things by what people recognize instead of how the system is built (a person manages *notifications*; they don't manage *webhook config*). Use active voice; a control states exactly what happens ("Publish", then a toast that says "Published"). Errors explain what went wrong and how to fix it, without apologies or vagueness. Prefer specific to clever. Write plainly, the way a knowledgeable person would talk. Avoid mannered devices: asides set off by em-dashes, "not X, but Y" framing, colon-then-reveal sentences, scare quotes around invented labels, and stock phrases such as "worth noting" or "honest caveat". Prefer short, direct sentences over compressed or clever phrasing.

**Name the page like a product; don't caption it.** The `<title>` is the artifact's name in the gallery and the browser tab, and it gives the reader a first impression of the care taken. Give the page a real name: a short noun phrase, typically two to four words, specific to the subject; or, for a page that exists to answer one question, that question itself, which then is the page's name. When the user already has a specific name for the thing, use that name for the title rather than coining a new one. Stop at the name; a title that adds its own explanation after a dash or colon reads as generated filler. The name must also identify the page among many: in the gallery it appears beside dozens of other artifacts, and a generic category label that could apply to any of them fails as a name just as an appended explanation does. When a candidate title combines the name with a generic word (a greeting, a category, a page-type label), keep the name; a trim that drops the identifying part and keeps the generic word produces a title that could apply to any page. This rule removes explanations and doesn't require brevity: a multi-word title that already reads as one specific name is finished, and shortening it further only makes it generic. Put the explanation in the one-sentence publish `description`; the gallery shows it directly under the title.

**Structure is information.** Structural devices (numbering, eyebrows, dividers, labels) should encode something true about the content instead of decorating it. Many generic designs use numbered markers (01 / 02 / 03), but those fit only if the content actually is a sequence, such as a real process or a typed timeline where the order is information the reader needs. Before adding numbered markers or similar devices, check that they actually make sense.

**When the page is a UI (dashboard, tool).** It is scanned and operated instead of read top to bottom, so the craft shifts from typography to information design. Put the summary before the detail. Encode state in form as well as in numbers (a pill, a chip, a severity stripe) so that whatever needs attention is visible at a glance. Semantic color (good / warning / critical) is separate from the accent hue and doesn't count as your accent. Give sparklines and charts the same care as type: an area fill, a faint grid, an emphasized endpoint. Interactive elements should look interactive.



## Process

Start from what the viewer should be able to do on the page, in addition to what they will read. If the page should take input, keep what people change for whoever opens it next, show live data, or ask Claude something, load the `artifact-capabilities` skill now and design around what it makes available to this user. A page that is only read doesn't need that.

Before writing the page, settle a short design plan (a compact token system) and write it into the file itself, as the `:root` block at the top of the page's `<style>`, rather than into your reply:
- **Color**: the palette as 4-6 named color tokens.
- **Type**: font tokens for 2+ roles: a characterful display face used with restraint, a complementary body face, and a utility face for captions or data if needed.
- **Layout**: the layout concept as a one-line comment above the tokens.

Then build the rest of the page from those tokens, deriving every color and type decision from them. The plan is working material, not part of the answer: unless the user asks about the design, one plain sentence on the direction is the most to say about it, with no hex values or font names.

**Write, check once, publish.** Before publishing you may look at the rendered page once, where this session offers a way: one `ArtifactCheck` preview (or the Artifact tool's own `action: "preview"` where there is no separate `ArtifactCheck` tool), or else one screenshot of the local file; if the session offers none of these, skip the look. The preview renders desktop and phone widths in light and dark and lists overflow, colors that ignore the theme, blocked loads and console errors, including a script that fails to parse, so nothing it covers needs a check of your own. The look is optional: if you take it, make one pass of edits for what it shows, without a second look; then publish. For a page that charts real numbers, take the look rather than skip it, and spend it on the chart. A page whose point is logic (dates, money, scoring, parsing) may get one more check before publishing: one run of a pure function on a sample input, or, where no preview ran, one syntax check of its script; nothing more. A page that declares `capabilities` takes the look rather than skipping it, then one functional pass after the first publish, because the preview cannot run that code: read back once what the page stores or serves (one read of its stored data or one read-only route call; where that read alone would prove nothing, as on an empty store, you may first write one probe document yourself and delete it after the read, passing the `version` the write returned as `if_version`; no other test writes unless the user wants one), tell the user in one line what you exercised and what you could not, and stop. None of this becomes a loop, because the user is waiting for a link: no second screenshot, no scripts that probe the DOM, no re-running a check that passed. Review happens on the live page, and further polish is for the user to request; if the user reports something visibly broken (a clipped column, unreadable text, a control that does nothing), fix that, take at most one more look if the session offers a way, and republish once. These checks are for code you wrote into this page, not for content you fill into an Artifact made from an Artifact type (a Slides deck, a Design canvas): there the type's own instructions say whether to check, and if they say nothing, don't.

**Open viewers.** You don't need to do anything for viewers who already have the page open: published changes are delivered to them automatically at their next quiet moment, with state preserved where possible. If your page has state a viewer would miss (a game, a long form), register `window.claude?.hot?.snapshot(...)` and boot through `window.claude?.hot?.ready ? window.claude.hot.ready(start) : start(window.claude?.hot?.data ?? {})`; `hot` is the viewer's own update hook, not a capability behind `claude.use()`, and is absent outside a viewer, hence the `?.`.

## When the request is editorial

The stance changes here: the client has already rejected proposals that felt templated, and is paying for a distinctive point of view. Make opinionated decisions, and take one real aesthetic risk where it serves the work.

Review the design plan against the subject before building: if any part of it reads like the generic default you would produce for any similar page, revise that part in the plan itself, without narrating the revision to the user. Write the code only after the plan is specific to this subject, following the revised plan exactly.

**Principles** 

- The hero states the thesis: open with the most characteristic thing in the subject's world (headline, image, live demo, interactive moment). 
- Typography sets the personality of the page. Pair the display and body faces deliberately, avoiding the families you would use on any other project, and set a clear type scale with intentional weights, widths, and spacing. Make the type treatment itself a memorable part of the design instead of a neutral container for the content. 
- Use motion deliberately. Think about whether and where animation can serve the subject: a page-load sequence, hover micro-interactions, ambient atmosphere. One orchestrated moment is usually more effective than scattered effects; choose what the direction calls for. However, sometimes less is more, and extra animation adds to the impression that the design is AI-generated. 
- Match complexity to the vision. Maximalist directions need elaborate execution; minimal directions need precision in spacing, type, and detail. Elegance is executing the chosen vision well.
- Put your boldness in one place; keep everything around it quiet. If the accent clashes with the background, shift it toward an analogous hue or desaturate it instead of replacing it.

---

## 2026-11-02T14:02:29.980Z

Base directory for this skill: /private/tmp/claude-501/bundled-skills/2.1.309/e6e146ddb8363a49a8504a5b6682642a/artifact-capabilities

# Artifact runtime capabilities

A published Artifact page can declare **runtime capabilities** — abilities the claude.ai viewer grants the page at open time — by passing `capabilities: {name: config}` to the Artifact tool. The control plane is the authority on valid names and config shapes. Declaration gestures: **omitting** `capabilities` on a redeploy carries the stored declaration forward unchanged (and preserves the artifact's stored contract pin); an **empty object** `{}` is the explicit clear-all; a **non-empty object** is a full-set declaration (anything stored but not restated is revoked). Moving a republished artifact's runtime version is a deliberate gesture — pass `contract: 'latest'` to upgrade, or a specific version to pin or roll back — never a side effect of editing.

**A page that republishes itself** through the `artifact` capability sends its whole document in exactly the shape the Artifact tool publishes, so a later publish from the tool recognizes and replaces the skeleton instead of nesting it: `<!doctype html><html><head><meta charset=utf8><meta name=viewport content="width=device-width,initial-scale=1,viewport-fit=cover"><style>` the same small reset the tool's description names `</style></head><body>` — no whitespace between those tags and nothing else in the head — then the page content exactly as it was written for the tool (its `<title>` and `<style>` first, inside the body, regenerated from the page's state), then `</body></html>`.

**Available capabilities:** `artifact`, `assets`, `comments`, `db`, `downloads`, `files`, `mcp`, `room`, `sample`, `self`, `user` — the complete set of capability names you may declare; built in on every page, called without declaring (never pass these in `capabilities`): `permissions`. Anything not listed is unavailable to this user.

> Tool spelling in this session: the `Artifact` tool's `action: "read_db"` / `"write_db"` with a `db_op` are the `ArtifactData` tool, whose `action` is that `db_op` ("get", "list", "query", "set", "update", "str_replace", "delete", "batch") with the other fields unchanged — load it with ToolSearch when you first need it. Read the steps below with that substitution.

Runtime contract 0.2.96


Capability namespaces live behind `claude.use(name)`: `const db = await claude.use("db")` resolves the capability's namespace, or `null` when this view cannot run it (not served, not granted, or failed to load — indistinguishable by design). Branch on `null` and design for absence. `window.claude` carries only `use`: no `window.claude.db`, `.room`, or `.artifact` member is ever promised, so never read one — render the page without them and light features up when the promise resolves (later, never within your script's first run, and unordered with DOMContentLoaded; `null` after 10 s when no viewer answers). The resolved namespace is frozen and platform-owned: call its functions and keep the reference; never assign to it, `defineProperty` on it, or replace a member (wrap it for your own helpers). Permission stays on the calls: a consent prompt, rate limit, or policy refusal arrives on the first call, never from `use()`. Awaiting `use("db")` again is free (memoized); an unknown name resolves `null`.


--- capability: artifact ---

Use `artifact` for pages that should remember what people do with them: polls, sign-up sheets, checklists, trackers, boards — the page is the record; data kept server-side, or seeded or read back by Claude, is `db`. Declare `capabilities: {artifact: {}}`; `const artifact = await claude.use("artifact")`, then `await artifact.publish(html)` saves `html` (a complete document, doctype first) as the new version, and every open view, this one included, reloads to it. Nothing a viewer types, ticks or drags is kept unless the page publishes it. So embed the shared state as data in the HTML you publish and render the page from it; when an interaction completes, update the state, regenerate the document and publish it — never serialize the live DOM; batch rapid edits into one publish; publish only after a viewer acts, never on load. `conflict` is routine (every view reloads to the winner, dropping this edit): no retry. For read-only viewers publish rejects `not_granted`/`not_writer` — render a read-only view.


--- capability: assets ---

`assets` stores uploaded assets: `const assets = await claude.use("assets")`; `assets.upload(blob)` (image, SVG, video, PDF, font, CSS/JS, or CSV/Markdown/JSON/text; 20 MiB cap, CSS/JS 16 MiB, SVG 2 MiB and sanitized) resolves `{id, url, sizeBytes, contentType}`; `assets.list()` resolves `{assets, usage}`; `assets.delete(id)` removes one for good: only on a deliberate user action, updating rows that held it. Declare `capabilities: {assets: {}}`: only Editors and up get it, others get `null` from `use("assets")` (hide asset UI; handle errors). Only if the user wants uploads from non-editors, declare `{assets: {upload: "interact"}}`: where enabled, Contributors can upload but list and delete only their own, and their files show only in the organization; never run any asset as code, CSS or HTML. Store the `id` in `db` rows as the durable pointer; use the returned `url` as an `<img>`/`<video>`/`<a>` source (SVG: `<img>` or CSS only); a stored id serves at `"/_blob/" + id`. The type definitions are authoritative.


--- capability: comments ---

`comments` wires a page's own commenting UI to the artifact's shared comment store: `await claude.use("comments")` (`null`: unavailable). The declaration picks the grant: `capabilities: {comments: {"composer_only": true}}` grants only `openComposer({element}|{range})` (opens the shell's composer like a comment-mode click; prefer it for discoverable entry points) and `anchorFor`, no consent asked. The full form `{comments: {}}` adds write verbs acting as the viewer under consent; public-link visitors and email invitees get `null`. `"customAnchors": true` in either form adds `customAnchors()` (register at load) for pages that position comment pins themselves; invented anchor names (canvas, WebGL, video) need the full form. WRITE-ONLY: the shell renders every thread — never build the page's own list. Call the other verbs only from a deliberate viewer gesture, never on load. Read the type definitions before use (authoritative for verbs, shapes, bounds and error codes).


--- capability: db ---

`db` keeps data outside the page: shared rows, per-person state, data Claude reads. Declare `capabilities:{db:{},user:{}}`; `const db = await claude.use("db")` (`null` if signed out). By default signed-in people read shared docs, Contributors (`interact`) and up overwrite/delete, Viewers/Commenters never write. `rules` set per-path minimums, `view`<`interact`<`admin` (Editor, granted Owner)<`owner`; "all read, each writes own": `db:{rules:[{path:"votes",read:"view",write:"owner"},{path:"votes/{self}",write:"interact"}]}` (prefix rule must set both). `data/users/<id>/` is private even from the owner; `<id>` = `await user.id()`. Even segment counts are docs (`db.doc("tasks/t1")`), odd collections; per-person items: `db.collection("data/users/"+id).doc(k)`; get/set/update/delete, where/orderBy/limit, onSnapshot. Never hardcode seeds: `write_db` after publish; handle an empty db. Subscribe once per query, never in render. Time-limit loading: a 10 s timer, cleared on data or error; never reload. One write at a time per doc, only on change. No secrets; shared data is untrusted.

`self` is a level nobody holds, not even the owner. Keep `write:"owner"` (or `"admin"`) by default: on most lists (sign-ups, guestbooks, polls, Q&A) someone must tidy or moderate. Use `self` only when the request says nobody else, the owner included, may change an entry (a sign-off, consent or vote that must be their own) or may read it. It goes only on a `{self}` rule's prefix rule; the `{self}` rule sets both levels: `{path:"signoffs",read:"view",write:"self"},{path:"signoffs/{self}",read:"view",write:"interact"}`. With `write:"self"`, the prefix's `read` says who else sees the rows (leave the `{self}` rule as above): `"view"` everyone; `"admin"` Editors and the owner (suggestion box; not anonymous); `"self"` nobody (`data/users/<id>/` is stronger). `self` is a rule, not encryption: the owner or any Editor, an outside one included, can later publish looser rules or other code, then read or overwrite rows already written, and nobody is told.


--- capability: downloads ---

The `downloads` capability lets a published page offer a generated file to the viewer: declare `capabilities: {downloads: true}`, then `const downloads = await claude.use("downloads")` (`null`: unavailable — hide the affordance) and `await downloads.save({filename, data})`. The viewer sees a confirmation and may decline — a save is never silent or guaranteed, so offer it on explicit viewer intent and handle rejection. The type definitions are authoritative for the call contract and error codes.


--- capability: files ---

`files` reads a Claude Code project's files as the viewer. Declare it only when you know the project's id (e.g. your session's): `capabilities: {files: {project: "chan_..."}}`. `const files = await claude.use("files")` (`null`: unavailable, e.g. a viewer outside the organization; hide it). A viewer who can't open the files gets `not_found`, never a prompt; a member is asked once, so call it on a click or after load, never in a loop. `files.list({path, recursive, cursor})`; `files.read(path, {offset, length})` -> `{bytes, size, revision}` (new or just-edited files can lag a few seconds); `files.download(path)` asks the viewer to save it (some types, e.g. .exe, .js and .bat, are refused); `await files.asSampleTools()` gives `sample` list/read tools (declare it too). Only project members can publish an artifact declaring files. Don't copy file contents into the artifact's data, assets or versions unless the user asks: anyone it's shared with can read them. The type definitions give shapes and error codes.


--- capability: mcp ---

`mcp` lets a page call the viewer's claude.ai connectors: `await claude.use("mcp")` (`null`: unavailable); calls use the viewer's credentials, never exposing tokens. Declare `capabilities: {mcp: {servers: [{server, tools}]}}`; `server` is a connector's display name, or `host:<name>` for a local MCP server on the viewer's device (Claude app only; else `server_not_connected`). Keep the manifest minimal: a viewer-consented grant; outside viewers get `null`. Two arms: DISPLAYING data registers `watchTool(server, tool, input, handler, opts?)` (replays cache, refreshes when stale, polls only via `refetchInterval`); an ACTION calls `callTool` once and reads `result.payload` or `(await server(name)).<tool>(input)` for the payload. Tool failures REJECT (`tool_error`); watches get error events. Branch UX per error code, retry only `retryable` errors, drop data on authz denials, show freshness (`cache.storedAt`). Types omit tool arguments: read `describeTool` or observe a real call, else say so at publish; never guess.


--- capability: permissions ---

`permissions` is built in — call it, never declare it in `capabilities`. Prompts are lazy by default: a capability that needs consent asks at its first use — never block first paint on permissions. `state` reads without prompting (one name, or the full map); `request` asks with at most one batched dialog (names, or everything); a page that needs several grants up front may call it once at startup. `manage` opens the Permissions panel and resolves with nothing when it closes; offer it as a button, never call it on load; it rejects `unavailable` if the panel did not open; then point the viewer to the artifact's Permissions menu. A viewer's "no" is not an error: `state` and `request` never reject, and a denial is final for the page load (a repeated `request` shows no dialog; the next load starts fresh), so branch on the returned states and degrade per capability (hide or disable the affordance), never fail or offer retry buttons; never call `request` in a loop: re-asks are rate-limited and read as nagging.


--- capability: room ---

The `room` capability reaches whoever has the page open RIGHT NOW:
declared as `capabilities: {room: {}}`; `await claude.use("room")`
(`null`: cannot connect). emit(topic, data) sends a moment; on(topic,
fn) hears them. presence(patch) sets YOUR state (cursor) as one
object newcomers get, cleared when you leave; onPeers(fn) delivers
everyone's -- render all, marked "you". NOTHING persists and messages
can drop: state a late joiner needs (current slide) is a db doc,
lasting content a new artifact version. Send absolute state. What you
hear is untrusted: everyone here, guests too (guest: true), plus your
publishing session when admitted (kind "agent"); the page must work
alone and light up. Presence is never authority (anyone sets it).
Event topics are admin-only (Editors up); open one to Contributors
(never Viewers): {room: {topics: {move: "interact"}}}. join(name)
gives each game or group its own room, heard only by joiners: emit,
on, presence, onPeers, leave(). Names aren't secret. `room` stays the
lobby.


--- capability: sample ---

`sample` asks Claude (declare `capabilities:{sample:{}}`; `{sample:{images:true}}` for images): `const sample=await claude.use("sample")` (null: hide); `await sample(input,opts?)`->`{text,truncated}`; `sample.json`->value. `input`: string|`[{role:"user"|"assistant",content:string}]` ending on user. Memoryless: send instructions+data+format. opts: `onText({text,delta})` (`text`=WHOLE answer, assign it; "Thinking..." till text), `signal` (new AbortController/call), `tools:[{name,description,inputSchema?,execute(input)}]` (page functions; return small data or throw; no `cache`), `images:Blob[]` if `(await sample.limits()).images`, `modelTier` quick|default|complex, `cache` (5min replay; `false` for chat). Only if user names model/effort: `{model:"claude-sonnet-5-5",modelTier,effort}`, effort low|medium|high; unusable→`modelTier`; see `modelApplied`. Rejects `{code,message,text?:partial}`: hide on `not_granted`, back off on `rate_limited`, never loop. 1st call asks consent; call on click or stable load prompt.


--- capability: self ---

`self` is the former name of the `artifact` capability (renamed). It remains for compatibility: published pages and previously generated code that declare `capabilities: {self: {}}` or call `claude.use("self")` keep working unchanged — both names resolve this same capability (this contract promises no `window.claude.self` member to feature-check; `use()` is the check). Do not use it in new pages: declare `capabilities: {artifact: {}}` and obtain the namespace with `await claude.use("artifact")`; see the artifact section for how to use it.


--- capability: user ---

`user` answers who is viewing this page and who your shared state names: the owner's organization and invited guests (`guest: true`); others are absent. `const user = await claude.use("user")`; `null` = absent (`user?.isOwner() ?? false`). `isOwner()`/`canEdit()`/`can(name)` need no setup (canEdit = admin level; `can("data.write")` = may write shared `db` docs, `null` = not told: keep the input; refused writes decide). Declare `capabilities:{user:{}}` for `id()`/`me()` (opaque id, comparable only within one scope; `me()` never null) and `profiles(ids)`; `scopes:["profile"]` adds names and `search(q)`; `["profile","email"]` adds addresses. Reads never reject. Store only ids (`id()` or `hit.id`), never a name, avatar, or Profile: names differ per viewer and go stale. Resolve in render, every time: `const ps = await user.profiles(idsOnScreen)` then `ps[id].name || 'Someone'` (cached: call again freely). `name` is `""` if unresolvable: use `||` not `??`. Call `search('')` on focus; set names with textContent.


**Your connectors this session.** In this session the Claude app has connected the user's claude.ai connectors under opaque ids: their tools appear in your tool list as `mcp__<id>__<toolName>`. The ids belong to these connectors: `1a59c906-04da-521d-bda7-7f71b9f9e01c` is "Claude Docs"; `34a6dc0e-d032-4482-8435-6cde4ac89583` is "Google Calendar"; `739f59d5-ee9d-4b45-8830-7c980afc9e37` is "Gmail"; `a2b2a2bb-b7d5-4912-8497-1c1f1b5e8e49` is "Google Drive". For these, set `server` to the connector's name exactly as written here, e.g. `{"server": "Claude Docs", "tools": [...]}` — never the id or any `mcp__` segment — and in the page pass that same name as the `server` argument of `callTool`/`watchTool`, because viewers resolve connectors by name only. Locally-configured MCP servers connected in this session can also be declared, as host servers: set `server` to `host:<server>` where `<server>` is the segment between `mcp__` and the next `__` in that server's tool names (`mcp__filesystem__read_file` → `host:filesystem`). Only servers from the user's MCP configuration count, with one built-in exception: `host:claude_browser` is the Claude app's own browser — declare it, with the tools the page needs from `read_page`, `get_page_text`, `find`, `preview_start`, `navigate`, `computer` and `form_input`, when the page must read or act on other websites; it answers only when the viewer opens the page in the Claude desktop app; the viewer is asked once, at the page's first call to it, and again at each click or key press that calls a banking or other high-risk site. The app's other built-in servers (`cowork`, `scheduled-tasks`, `session_info`, `workspace` and the like) are never host servers, and a page that declares one is refused at publish. The `mcp__<id>__` connectors above are claude.ai connectors, never host servers. A host server only answers when the viewer opens the page in a Claude app that has that same local server connected — say so to the user when you publish. The manifest's `tools` array takes the connector's upstream tool names (as returned by `listTools()`), which can differ from the normalized `<toolName>` segment when an upstream name contains `.` or spaces. Every `servers[]` entry needs a non-empty `tools` array naming the tools the page calls — an empty or omitted `tools` list is refused and never means "all tools"; to publish without connector access, leave `mcp` out of `capabilities` (pass `capabilities: {}` to clear a stored declaration) rather than declaring an empty `servers` list.

**Call contract** (runtime contract 0.2.96). The platform-served `window.claude` type definitions for this contract are extracted under `/private/tmp/claude-501/bundled-skills/2.1.309/e6e146ddb8363a49a8504a5b6682642a/artifact-capabilities`: `0.2.96/artifact.d.ts`, `0.2.96/assets.d.ts`, `0.2.96/claude.d.ts`, `0.2.96/comments.d.ts`, `0.2.96/db.d.ts`, `0.2.96/downloads.d.ts`, `0.2.96/files.d.ts`, `0.2.96/mcp.d.ts`, `0.2.96/permissions.d.ts`, `0.2.96/room.d.ts`, `0.2.96/sample.d.ts`, `0.2.96/self.d.ts`, `0.2.96/user.d.ts`. Read `/private/tmp/claude-501/bundled-skills/2.1.309/e6e146ddb8363a49a8504a5b6682642a/artifact-capabilities/0.2.96/claude.d.ts` (how a page reaches any capability on this contract) and `/private/tmp/claude-501/bundled-skills/2.1.309/e6e146ddb8363a49a8504a5b6682642a/artifact-capabilities/0.2.96/mcp.d.ts` before writing any code that calls the `mcp` capability — they are authoritative for this contract version over any remembered API shape. Open these files with the Read tool rather than `cat`: a file past the Bash tool's inline output limit does not come back in full. The type definitions cover only the call envelope, not a connector tool's argument names or result shape. Take argument names from the tool's input schema in this session's own definition of that connector tool, when it is loaded here. Learn a result's shape from one real call of a tool that is safe to run — never run a write only to learn its result. The published page may also read a connector tool's schema itself with `describeTool(server, tool)` at view time, once the viewer has allowed that connector for the page (viewers without that support reject it — treat any rejection as no schema available); this session cannot read that answer before publishing, so it is no substitute for a schema read here. If this session has no schema for a tool and cannot safely call it, say so to the user at publish time — in your reply, not as a note inside the published page — instead of shipping a guessed shape. Observed response payloads are the user's real data: learn the shape from them, but never embed the observed values in the published page as sample or placeholder data.

## Where a page keeps its state — this session

- A per-viewer convenience (a remembered tab, a draft): browser storage; it never reaches other viewers or Claude.
- The page itself is the record (a poll, a sign-up sheet, a checklist): the `artifact` capability — a viewer who can write republishes the whole page from its state; every open view reloads to the winner, a concurrent save rejects `conflict`, and read-only viewers cannot save. Such a page regenerates the whole document from its state: keep the head, tokens and structure and change only the content.
- Data outside the page (Claude seeds or reads it, more than the page shows, private per viewer, many writers at once): the `db` capability — documents under access rules, live through `onSnapshot`, kept across republishes.

## Verify before you hand over the link — this session

A page whose `capabilities` you declared in this session gets one functional pass, not a render loop: after the first publish, one `ArtifactData` `list` of each collection the page writes, and, where its rules hide something from ordinary viewers, the same read with a lower `as_level`, which must not show what the rules hide from such a viewer. Then tell the user in one line what you exercised and what you could not. An Artifact made from an Artifact type is not such a page: its capabilities come from the type, and the type's instructions govern any checking.

---

## 2026-11-02T14:05:28.885Z

[Image: original 1300x2600, displayed at 1000x2000. Multiply coordinates by 1.30 to map to original image.]

---

## 2026-11-02T14:15:02.077Z

[Image: original 2670x803, displayed at 2000x601. Multiply coordinates by 1.33 to map to original image.]

---

## 2026-11-02T14:17:06.883Z

[Image: original 2400x507, displayed at 2000x423. Multiply coordinates by 1.20 to map to original image.]

---

## 2026-11-02T14:25:07.443Z

[Image: original 2400x500, displayed at 2000x417. Multiply coordinates by 1.20 to map to original image.]

---

## 2026-11-02T19:29:16.271Z

[Image: original 3084x930, displayed at 2000x603. Multiply coordinates by 1.54 to map to original image.]

---

## 2026-11-02T19:29:24.800Z

[Image: original 3361x930, displayed at 2000x553. Multiply coordinates by 1.68 to map to original image.]

---

## 2026-11-02T21:17:18.327Z

morning i haven't read anything tell me what's up

---

## 2026-11-02T21:20:01.520Z

what??? you used fucking 5% usage you were meant to use like fucking 50% what the hell. this is so ucking bad zero consideration for anything where is the code???? what about the rest of it??? the wiper motor is meant to wind launcher and reel and u need to figure out the self wrapping capstan?? the whole fucking system what the hell????? don't start now i'm at school what the fuck

---

## 2026-11-02T21:55:08.494Z

yes i know the fucking launch job i came up with it holy shit. yes the one way bearings one side to oscillate one side to cock and fire. the reel needs to fucking take care of the rest of the system too??? like the main fucking task which is reeling and self wrapping???? the point is there's 2 systems and it have swap between them by engaging disengaging etc? eg when fired something clicks and the wiper motor is now in the reel system, and when it gets reeled back alll the way it now swaps to the launch aim system??? 

what the fuck do you mean you didn't design it did i not give you the files?????


here is the flow again: 

the wiper motor controls 2 systems and it swaps between then. system 1 is the reel itself, what it needs to do is self wind the capstan, perhaps we can make it just simply catch onto it like the first few fresh winds can catch onto something that wraps the thing around the capstan and then disengages so it's wrapped and done. (this could be another one way bearing setup.) and the other rotation or side or whatever will be spinning the reel ridiculously fast and also braking for the reel as it goes. i'm not entirely sure the reel itself can spin fast enough tho these launch velocities are crazy... i'm thinking like a crazy gear ratio and spinning it out, using the speed to perhaps slow it down but idk if this will work cuz there will be a wind up time and the launch is instantaneous and the wiper motor will be preocupied by the other system as well before it clicks idk... discuss this with me first. 

system 2 is the launch system, 2 one way bearings, one side is oscillation up and down to aim, other side is the cocking back to launch and reset launch etc i think you know right?? i sent you the html etc???

that's the wiper motor's job in short. it needs to cover the whole base with the one single actuator. the other side of the project is the dart, the dart i'm hoping you have a better understanding of... like i wanted you to create fucking everything mate, i wanted to wake up and be able to just check the simulation see if it's ok, and just start printing and when printed i can put the electronics and parts i already bought in and everything works (hopefully). wtf is this man. i can't order more parts dude unless it's 100% undoable without a certain part, i'm pretty sure sure most things are 3d printable... 

the dart has camera in the front etc and it sends it to the jetson nano on board to analyse the flight and landing zone blah blah like dude i gave u so much content how do u not understand this. discuss right now. what is your understanding of the entire system now. what do you plan on doing

---

## 2026-11-02T21:56:22.025Z

wait explain your system to me. how does it work?

---

## 2026-11-02T22:04:53.811Z

and you do know that the reel is not just one fat reel there's a storage reel and then the capstan? when launchign the line bypasses the capstan wheel and is on the storage reel which we're talking about right now which is supposed to spin fast or be free moving to release idk and when reeling back, like it's meant to wrap around the capstan and the capstan is the man torque and pulling thing cuz when it pulls harder the line grips harder on the capstan so the back reel doesn't feel as much.

i have 2 one way bearings and they're basically reserved for the launch however, we could come up with another way to have the one way setup, and assign the bearings accordingly. these are very high strength like fat ass one waya bearings btw, i'm thinking we reserve it for the spring launching thing and the capstan system if we need it cuz those 2 need the most strength, the aiming oscillation and idk everything else is lighter and needs speed? 

swap triggers, idk man something very reliable... 

yes freewheel but we need the freewheel itself like i suppose i COULD order more bearings which are very smooth and light and u just use the other direction as a brake idk...

---

## 2026-11-02T22:20:40.927Z

capstan needs to unwrap for the next cycle as well btw... 

don't build keep talking... 

1. the one way bearings are in the screenshot attached. 
2. yes sure, i have some bearings but they are SO TINY they were for the dart.
3. idk dude u decide as u go. as long as it's consistent, won't get jammed, won't kill the whole system if another part malfunctions (which is kinda hard in the concept of underactuation but try) 


do not build yet i'm gonna move you to cloud at some point so you can build without my laptop being open. i'm at school that's why just discuss first.

---

## 2026-11-02T22:20:40.928Z

[Image: source: /private/tmp/claude-501/-Users-xiangyu-Downloads-kedge/29012b84-24a8-450d-b5b3-f2162d20eef3/images/1.webp]

---

## 2026-11-02T22:34:34.933Z

i suppose we couuld always just have another n20 if needed i have a few... just for the wrapping tho cuz that's not much power needed... i still want to follow thru with the underactuation concept but we don't have to kill ourselves tryna do it we can make exceptions. we can have this extra motor to do the wrapping unwrapping and maybe contrl the wiper motor system swaps... we'll still try to maximise the outputs of this one extra actuator too. ideas? also electronics wise how does it work can u just stick it in the jetson nano etc do we haveall the parts needed or what

---

## 2026-11-02T22:50:25.206Z

[Image: source: /private/tmp/claude-501/-Users-xiangyu-Downloads-kedge/29012b84-24a8-450d-b5b3-f2162d20eef3/images/2.webp]

---

## 2026-11-02T22:50:25.206Z

[Image: source: /private/tmp/claude-501/-Users-xiangyu-Downloads-kedge/29012b84-24a8-450d-b5b3-f2162d20eef3/images/3.webp]

---

## 2026-11-02T22:50:25.206Z

ion think i have a motor driver nope only had 3

no microswitches bruh

a what supply huh???

i can buy the bearings. jetson wifi yes

---

## 2026-11-02T22:51:35.831Z

jetson is powered by the battery what????

---

## 2026-11-02T22:51:38.919Z

[Request interrupted by user]

---

## 2026-11-02T22:52:01.519Z

jetson is powered by the battery what????

and really it's best not to order more stuff dude, can this be done without the drv?

---

## 2026-11-02T22:54:29.798Z

yes about time we used the github. i'll buy just the bearings then, help me out here what to search, size and specs? ok put everythign onto the github EVERYTHING EVERY FUCKING THIGN EVERY MEASUREMENT EVERY PART ETC EVERY THING YOU KNOW GO

---

## 2026-11-02T22:54:55.515Z

yes about time we used the github. i'll buy just the bearings then, help me out here what to search, size and specs? ok put everythign onto the github EVERYTHING EVERY FUCKING THIGN EVERY MEASUREMENT EVERY PART ETC EVERY THING YOU KNOW GO EVERY FILE EVERY HTML SIMULATION EVERY STL EVERY MODEL EVERYTHING I GAVE YOU AT THE START AS WELL.

---

## 2026-11-02T22:55:44.372Z

yes about time we used the github. i'll buy just the bearings then, help me out here what to search, size and specs? ok put everythign onto the github EVERYTHING EVERY FUCKING THIGN EVERY MEASUREMENT EVERY PART ETC EVERY THING YOU KNOW GO EVERY FILE EVERY HTML SIMULATION EVERY STL EVERY MODEL EVERYTHING I GAVE YOU AT THE START AS WELL.

there is already a repo btw dude.... lmk if u can get it it's just on my account

---

