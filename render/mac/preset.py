"""Run inside Blender on the Mac: sets the quality preset and renders the mission.
   Blender -b grappstar_mission.blend -P preset.py -- --preset uhd --engine eevee [--start N --end M] --out /path/frames/
Presets: final (1920x1080, the default: reads most like real footage), uhd (3840x2160), preview (1280x720,
quick look), hero (3840x2160 Cycles quality, for stills). The .blend carries the camera look (AgX, 180 deg shutter motion blur,
sun veil and lens ghosts, bloom, lens distortion and chromatic aberration, softness with a phone-style sharpening
halo, vignette, sensor noise in the compositor) and the two locked-off cameras with their cut. The MP4 is H.264 at
CRF 23 (a modest bitrate, like a phone).
Frames are PNG (resumable: frames already rendered are skipped). Then they are joined into an MP4 in the same folder.

Rough time per frame on an M3 (10-core GPU, 16 GB), EEVEE, this scene (about 30 M instanced grass triangles,
screen-space ray tracing, soft shadows, motion blur): preview 3-6 s, final 8-15 s, uhd 25-45 s (4x the pixels of
final; the grass geometry cost stays the same). The first frame also compiles shaders (a few minutes). For the
whole film multiply by the frame count in the timeline (frame_end + 1). If memory runs short at uhd, lower the grass
Density (env:grass > Modifiers > scatter > Density, e.g. 0.7)."""
import bpy, sys, os, argparse

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--preset", default="final", choices=["preview", "final", "uhd", "hero"])
ap.add_argument("--engine", default="eevee", choices=["eevee", "cycles"])
ap.add_argument("--start", type=int); ap.add_argument("--end", type=int)
ap.add_argument("--out", default=os.path.expanduser("~/Desktop/grappstar_render/frames/"))
ap.add_argument("--no-video", action="store_true")
a = ap.parse_args(argv)

sc = bpy.context.scene
r = sc.render
res = {"preview": (1280, 720), "final": (1920, 1080), "uhd": (3840, 2160), "hero": (3840, 2160)}[a.preset]
r.resolution_x, r.resolution_y, r.resolution_percentage = res[0], res[1], 100
if a.start is not None: sc.frame_start = a.start
if a.end is not None: sc.frame_end = a.end
r.use_motion_blur = True
r.motion_blur_shutter = 0.5                                   # 180 deg shutter
r.use_compositing = True                                      # lens distortion, vignette, grain

if a.engine == "cycles":
    try:
        bpy.ops.preferences.addon_enable(module="cycles")
    except Exception:
        pass
    r.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"                      # Apple Silicon GPU
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
    sc.cycles.samples = {"preview": 24, "final": 96, "uhd": 128, "hero": 512}[a.preset]
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.use_denoising = True
else:
    r.engine = "BLENDER_EEVEE"
    ee = sc.eevee
    ee.taa_render_samples = {"preview": 16, "final": 48, "uhd": 48, "hero": 128}[a.preset]
    if hasattr(ee, "use_raytracing"):
        ee.use_raytracing = a.preset != "preview"

os.makedirs(a.out, exist_ok=True)
r.filepath = os.path.join(a.out, "f_")
if hasattr(r.image_settings, "media_type"):
    r.image_settings.media_type = "IMAGE"
r.image_settings.file_format = "PNG"
r.use_overwrite = False                                       # resume: skip frames already on disk
r.use_placeholder = True
print(f"GrappStar render: {a.preset} {a.engine} {res[0]}x{res[1]} frames {sc.frame_start}-{sc.frame_end} -> {a.out}")
bpy.ops.render.render(animation=True)

if not a.no_video:                                            # join the PNGs into an MP4 (H.264) next to them
    fps = sc.render.fps
    vs = bpy.data.scenes.new("video")
    vs.render.resolution_x, vs.render.resolution_y, vs.render.resolution_percentage = res[0], res[1], 100
    vs.render.fps = fps
    vs.sequence_editor_create()
    files = sorted(f for f in os.listdir(a.out) if f.startswith("f_") and f.endswith(".png"))
    if files:
        strip = vs.sequence_editor.strips.new_image("frames", os.path.join(a.out, files[0]), 1, 1) \
            if hasattr(vs.sequence_editor, "strips") else vs.sequence_editor.sequences.new_image("frames", os.path.join(a.out, files[0]), 1, 1)
        for f in files[1:]:
            strip.elements.append(f)
        vs.frame_start, vs.frame_end = 1, len(files)
        if hasattr(vs.render.image_settings, "media_type"):
            vs.render.image_settings.media_type = "VIDEO"           # Blender 5: video output is its own media type
        vs.render.image_settings.file_format = "FFMPEG"
        vs.render.ffmpeg.format = "MPEG4"
        vs.render.ffmpeg.codec = "H264"
        vs.render.ffmpeg.constant_rate_factor = "MEDIUM"                 # CRF 23: modest bitrate, real compression texture
        vs.render.filepath = os.path.join(a.out, "grappstar_mission.mp4")
        bpy.ops.render.render(animation=True, scene=vs.name)
        print("video:", vs.render.filepath)
