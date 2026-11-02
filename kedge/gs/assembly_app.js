// GrappStar ground station: step by step assembly, parts fly in from their insertion direction.
(function () {
  const GS = window.GS, P = GS.P, $ = s => document.querySelector(s);
  const V = GSV.build($('#view'), { hd: true });
  const { meshes, cam, ctl } = V;
  const STEPS = [
    { t: 'Base tiles', d: 'Heat-set M4 inserts in every frame foot hole first. The five base tiles bolt to the rover deck before anything else, 4 x M4 cap screws each, heads flush in the counterbores.', parts: ['base_1', 'base_2', 'base_3', 'base_4', 'base_5'], off: [0, 0, -120], cam: [[650, -900, 700], [100, 30, 0]] },
    { t: 'Frame', d: '608ZZ cam roller with an M8 washer each side into the post on the right plate, 8 x 22 pin pushed in from the inside. 6001-2RS in each plate. Plates, bumper posts (16 x 15 bumpers on M6 studs into inserts), motor plate, saddle, electronics box, trip post and strap post screw into the tile inserts (M4). Each post: eye bolt through the shelf, height nut above, preloaded die spring below. AS5600 bracket onto the left plate (4 x M3).', parts: ['frame_plate_R', 'frame_plate_L', 'brg_frame_R', 'brg_frame_L', 'cam_roller', 'roller_pin', 'bumper_posts', 'rubber_foot_R', 'rubber_foot_L', 'motor_plate', 'motor_saddle', 'ebox', 'as5600_bracket', 'trip_anchor', 'trip_eye', 'trip_spring', 'strap_anchor', 'strap_eye', 'strap_spring'], off: [0, 0, 160], cam: [[700, -900, 650], [100, 30, 80]] },
    { t: 'Cheeks and bearings', d: 'Each cheek prints flat. Pivot and program gear 6001s, one backstop HF1216 C in each boss (in its steel sleeve, check the direction first). The tie bolts between the cheeks (2 x M4 x 20 per side into nuts), the spring anchor bar behind the pivot the same way. Press the check strap pin into the right cheek.', parts: ['cheek_L', 'cheek_R', 'cheek_tie', 'strap_pin', 'brg_pivot_R', 'brg_pivot_L', 'brg_pg_R', 'brg_pg_L', 'owc_C', 'spring_anchor'], off: [0, 0, 220], cam: [[380, -520, 420], [40, 0, 90]] },
    { t: 'Program gear', d: 'Stub through the left bearing, left spacer, the flange hub with the program gear bolted on (4 x M4 into its inserts, M5 set screws on their flats), right spacer, right bearing and C; the lift cam bolts to the second flange hub at the right end.', parts: ['pg_stub', 'pg_spacer_L', 'pg_hub', 'program_gear', 'pg_spacer_R', 'lift_cam', 'cam_hub'], xr: ['cheek_L', 'cheek_R', 'frame_plate_L', 'frame_plate_R'], off: [0, -180, 0], cam: [[420, -480, 300], [60, 0, 90]] },
    { t: 'Sear into the pocket', d: 'Before the rail goes on: grooved pin pressed into the boss on the sear outer plate, spring post into the left cheek pocket (2 x M3 x 10 from outside the cheek), sear spring on its 2 mm pins, sear standing in the pocket with its pin out through the arc slot.', parts: ['sear_spring_post', 'sear', 'cord_pin', 'sear_spring'], xr: ['cheek_L', 'frame_plate_L'], off: [0, 0, 120], cam: [[260, -420, 140], [90, -30, 120]] },
    { t: 'Carriage and rack', d: 'Rack under the carriage stem: one M3 x 35 down from the top of the carriage and a 3 x 12 dowel. The carriage prints on its side so the keel bends along its layers.', parts: ['rack', 'sled'], off: [-260, 0, 0], cam: [[420, -560, 380], [120, 0, 150]] },
    { t: 'Rail', d: 'B onto the tie post, A onto the anchor bar post (2 x M3 countersunk each). Program gear gap to the rack, slide the carriage into B from the front. Only then bolt C on at the flange (7 x M4) and fit the cheek tabs: 3 x M4 x 8 per side into the wall strip inserts. SW_SEATED on its web under the cradle.', parts: ['rail_beam', 'sw_seated', 'stop_pad_R', 'stop_pad_L'], off: [0, 0, 200], cam: [[700, -800, 600], [160, 0, 150]] },
    { t: 'Sear pivot and trip cord', d: 'M4 nut into the trap in the lug, shoulder screw 5 x 25 from outside through cheek, plates and into the nut (threadlocker, sear swings free). SW_COCKED outside the cheek (2 x M2). Trip cord: clove hitch on the pin behind the e-clip, down to the eye bolt; set the height nut so the hook clears at the release angle.', parts: ['sear_pin', 'sw_cocked'], xr: ['cheek_L', 'frame_plate_L', 'rail_beam'], off: [0, -120, 0], cam: [[260, -420, 140], [90, -30, 120]] },
    { t: 'Pulleys and springs', d: '2 x F693ZZ in each flanged pulley, M3 x 35 from outside the side wall through the forks, nyloc inside. Cord: carriage shoe, over the pulley groove, to the spring (screw-in end, foam surge damper inside); spring end on the rear pin in the anchor bar. 40 mm 90A urethane stops into their pockets. Check strap from the right cheek pin to the strap post, tight at 36 deg.', parts: ['pulley_R', 'pulley_L', 'spring_env_R', 'spring_env_L', 'cord_env_R', 'cord_env_L'], off: [0, 0, 160], cam: [[700, -700, 400], [280, 0, 120]] },
    { t: 'Reel', d: 'O-ring in the spool groove, spool onto the bracket (3 x M3), drag disc over the O-ring, pickup pin in the rotor with its peg in the disc slot. Stub through bracket and spool, rotor in front, stub gear behind (grubs on the D flats). Glue the ceramic eye ring into the cover, cover on (2 x M3). Bracket to the left cheek pad now (3 x M4 into inserts).', parts: ['reel_bracket', 'spool', 'oring', 'drag_disc', 'pickup_pin', 'reel_stub', 'rotor', 'reel_stub_gear', 'reel_cover', 'eye_ring'], off: [0, -200, 0], cam: [[-160, -460, 360], [-20, -40, 180]] },
    { t: 'Main shaft', d: 'In from the right: right plate bearing, right spacer, right pivot bearing, pinion spacer, drive pinion (HF1216 A inside), pinion spacer, left pivot bearing, middle spacer, reel main gear, its spacer, left plate bearing. Coupler on the D flat outside the right plate (2 x M4 grubs), main gear grub on its spot flat. Magnet in its 3 mm cap on the shaft end, AS5600 board.', xr: ['frame_plate_L', 'frame_plate_R', 'cheek_L', 'cheek_R'], parts: ['main_shaft', 'shaft_spacer_R', 'shaft_spacer_M', 'pinion_spacer_R', 'owc_A', 'drive_pinion', 'pinion_spacer_L', 'reel_main_gear', 'shaft_spacer_L', 'wiper_coupler', 'as5600_magnet', 'as5600_board'], off: [0, 260, 0], cam: [[300, -700, 320], [0, 0, 80]] },
    { t: 'Wiper motor', d: 'Its bolts in the motor plate slots, its crank arm bolted to the coupler, a hose clamp over the can in the saddle.', parts: ['wiper_motor'], off: [0, 200, 0], cam: [[300, 700, 400], [0, 60, 80]] },
    { t: 'Electronics', d: 'BTS7960 and ESP32-S3 in the box, SW_LOW on the right bumper post boss (2 x M2). Wire per the guide, LiPo last. Then the lid.', parts: ['bts7960', 'esp32s3', 'sw_low', 'ebox_lid'], off: [0, 0, 150], cam: [[600, -500, 400], [260, 0, 40]] },
    { t: 'Dart', d: 'Thread the tether from the spool, over the rotor skirt, out of the eye, through the carriage pusher hole, tie it to the dart tail holes. Cock, seat the dart. Ready.', parts: ['dart_body_hull', 'dart_body_elec', 'dart_kick', 'dart_kick_aug', 'dart_drillL', 'dart_drillL_aug', 'dart_drillR', 'dart_drillR_aug', 'dart_cpl'], off: [300, 0, 0], cam: [[600, -700, 500], [140, 0, 150]] },
  ];
  const assigned = new Set(STEPS.flatMap(s => s.parts));
  const phi = (P.PH_REL + P.PH_LIFT0) / 2, th = GSV.thetaOf(phi), s = GSV.sledPos(phi, false), alpha = th - phi * P.Z_PG / P.Z_PIN;
  const F = V.frames({ theta: th, phi, alpha, s, rot: P.STUB_PHASE - (alpha - th) * P.Z_MAIN / P.Z_STUB, drag: 0, pin: 0, sear: 0, kick: 0, dep: 0 });
  for (const k in meshes) if (!assigned.has(k)) meshes[k].visible = false;
  const list = $('#steps');
  STEPS.forEach((st, i) => { const b = document.createElement('button'); b.textContent = (i + 1) + '. ' + st.t; b.onclick = () => go(i); list.appendChild(b); });
  let cur = 0, anim = 0, t0 = performance.now();
  const cp = new THREE.Vector3(), ct = new THREE.Vector3();
  function go(i) { cur = Math.max(0, Math.min(STEPS.length - 1, i)); t0 = performance.now(); anim = 1;
    $('#st').textContent = (cur + 1) + ' / ' + STEPS.length + ' · ' + STEPS[cur].t; $('#sd').textContent = STEPS[cur].d;
    [...list.children].forEach((b, j) => b.classList.toggle('on', j === cur));
    $('#plist').textContent = STEPS[cur].parts.map(n => n.replace(/_env/, '')).join(', ');
  }
  $('#prev').onclick = () => go(cur - 1); $('#next').onclick = () => go(cur + 1);
  document.addEventListener('keydown', e => { if (e.key === 'ArrowRight') go(cur + 1); if (e.key === 'ArrowLeft') go(cur - 1); });
  let userCam = false; ctl.addEventListener('start', () => userCam = true);
  function frame(now) {
    const f = Math.min(1, (now - t0) / 1400), e = 1 - Math.pow(1 - f, 3);
    const xr = new Set(STEPS[cur].xr || []);
    for (const k in meshes) { const m = meshes[k], on = xr.has(k); if (m.material.transparent !== on) { m.material.transparent = on; m.material.opacity = on ? 0.22 : 1; m.material.depthWrite = !on; m.material.needsUpdate = true; } }
    STEPS.forEach((st, i) => {
      for (const n of st.parts) {
        const m = meshes[n]; if (!m) continue;
        m.visible = i <= cur;
        const base = F[m.userData.frame] || F.world;
        if (i === cur) { const o = new THREE.Vector3(...st.off).multiplyScalar(1 - e); m.matrix.copy(new THREE.Matrix4().makeTranslation(o.x, o.y, o.z).multiply(base)); }
        else m.matrix.copy(base);
        m.material.emissive = new THREE.Color(i === cur ? 0x553300 : 0x000000); m.material.emissiveIntensity = i === cur ? 0.35 * (1 - f) + 0.12 : 0;
      }
    });
    if (!userCam) { const c = STEPS[cur].cam; cp.lerp(new THREE.Vector3(...c[0]), 0.06); ct.lerp(new THREE.Vector3(...c[1]), 0.06); cam.position.copy(cp); ctl.target.copy(ct); }
    ctl.update(); V.render(); requestAnimationFrame(frame);
  }
  cp.set(...STEPS[0].cam[0]); ct.set(...STEPS[0].cam[1]);
  $('#recam').onclick = () => { userCam = false; };
  go(0); requestAnimationFrame(frame);
  window.ASM_GO = go; window.READY = true;
})();
