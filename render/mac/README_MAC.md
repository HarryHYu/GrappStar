# Render the GrappStar mission on your Mac

1. Install Blender 5.3 for Apple Silicon from blender.org (drag into Applications, open it once).
2. Put these files in `~/Desktop/grappstar_render/`:
   - `grappstar_mission.blend` (the whole scene: models, paddock, motion, cameras; textures packed inside)
   - `render_mac.command` and `preset.py` (this folder)
3. To LOOK at it: double-click `grappstar_mission.blend`, press Space to play (viewport), or scrub the timeline.
4. To RENDER the video: double-click `render_mac.command` (first time: right-click > Open, macOS asks once).
   It renders 1920x1080 ("final", the default: looks most like real phone footage) frames with EEVEE into
   `frames/`, then joins them into `frames/grappstar_mission.mp4` (H.264, CRF 23). Expect roughly 8-15 s per frame
   on an M3 16 GB (25-45 s at uhd 3840x2160), plus a few minutes of shader compiling on the first frame.
   Keep the lid open and the charger in (it runs `caffeinate` so the Mac does not sleep).
   Stopped half way? Run it again: frames already done are skipped.
   Other options (Terminal, in that folder):
   - 4K: `bash render_mac.command uhd`
   - quick test: `bash render_mac.command preview eevee 500 560`
   - photoreal stills or a short clip with Cycles on the GPU: `bash render_mac.command hero cycles 520 520`
5. Quality knobs: the presets in `preset.py` (resolution, samples). Grass density: select `env:grass`, Modifiers,
   scatter, Density (1.0 full; 0.7 saves memory and time). The camera look (lens distortion, vignette, grain) is
   the compositor node group GS_camera_look (render/BUILD.md).
