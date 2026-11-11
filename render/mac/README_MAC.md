# Render the GrappStar mission on your Mac

1. Install Blender 5.3 for Apple Silicon from blender.org (drag into Applications, open it once).
2. Put these files in `~/Desktop/grappstar_render/`:
   - `grappstar_mission.blend` (the whole scene: models, paddock, motion, cameras; textures packed inside)
   - `render_mac.command` and `preset.py` (this folder)
3. To LOOK at it: double-click `grappstar_mission.blend`, press Space to play (viewport), or scrub the timeline.
4. To RENDER the video: double-click `render_mac.command` (first time: right-click > Open, macOS asks once).
   It renders 1920x1080 frames with EEVEE into `frames/`, then joins them into `frames/grappstar_mission.mp4`.
   Keep the lid open and the charger in (it runs `caffeinate` so the Mac does not sleep).
   Stopped half way? Run it again: frames already done are skipped.
   Other options (Terminal, in that folder):
   - quick test: `bash render_mac.command preview eevee 1 120`
   - photoreal stills or a short clip with Cycles on the GPU: `bash render_mac.command hero cycles 900 960`
5. Quality knobs: the preset block at the top of `preset.py` (resolution, samples). Grass density is in the scene's
   "GrappStar quality" settings (see the build notes).
