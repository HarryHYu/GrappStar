"""Run inside Blender on the Mac: sets the quality preset and renders the mission.
   Blender -b grappstar_mission.blend -P preset.py -- --preset hd720 --engine eevee [--start N --end M] --out /path/frames/
Presets: hd720 (1280x720, the default: fast, reads like phone footage), final (1920x1080), uhd (3840x2160),
preview (960x540, quick look), hero (1920x1080 Cycles quality, for stills).
The film is the frame ranges stored in the .blend (scene property grappstar_ranges, e.g. "80-712,855-1322": the
idle before the aim and the base's wrap tail after the cut are skipped); they are rendered back to back and joined
into one MP4 (H.264, CRF 23: a modest bitrate, like a phone). --start/--end render one plain range instead.
The .blend carries the camera look (AgX Punchy, warm white balance, 180 deg shutter motion blur, sun veil and lens
ghosts, bloom, lens distortion and chromatic aberration, softness with a phone-style sharpening halo, vignette,
sensor noise in the compositor) and the two locked-off cameras with their cut.
Frames are PNG (resumable: frames already rendered are skipped).

Rough time per frame on an M3 (10-core GPU, 16 GB), EEVEE, this scene: hd720 a few seconds (the .blend's EEVEE is
tuned for speed: 20 samples, screen-space tracing at half resolution, 1 motion blur step, grass Density 0.8),
final about twice that, uhd 4-6x. The first frame also compiles shaders (a few minutes)."""
import bpy, sys, os, argparse

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--preset", default="hd720", choices=["preview", "hd720", "cy720", "final", "uhd", "hero"])
ap.add_argument("--engine", default="eevee", choices=["eevee", "cycles"])
ap.add_argument("--start", type=int); ap.add_argument("--end", type=int)
ap.add_argument("--out", default=os.path.expanduser("~/Desktop/grappstar_render/frames/"))
ap.add_argument("--no-video", action="store_true")
a = ap.parse_args(argv)

sc = bpy.context.scene
r = sc.render
res = {"preview": (960, 540), "hd720": (1280, 720), "cy720": (1280, 720), "final": (1920, 1080), "uhd": (3840, 2160),
       "hero": (1920, 1080)}[a.preset]
CY = a.preset == "cy720"            # Cycles on the Apple GPU at 720p with the .blend's (toned-down) camera look
r.resolution_x, r.resolution_y, r.resolution_percentage = res[0], res[1], 100
r.use_motion_blur = True
r.motion_blur_shutter = 0.5                                   # 180 deg shutter (opens at the frame: see the .blend)
if CY:
    r.use_compositing = True                                  # the .blend's camera look (neutral grade, light glare)
else:
    r.use_compositing = False                                     # clean image: no phone filter (Harry: too murky)
    _v = sc.view_settings
    try:
        _v.use_white_balance = False                              # no yellow cast
        _v.look = "None"
    except Exception:
        pass

# Harry: the grass must stay still. No drifting cloud shadows, no wind sway (grass and gum leaves).
_cs = bpy.data.objects.get("env:cloud_shadows")
if _cs:
    _cs.hide_render = True
    _cs.hide_viewport = True
_nw = 0
for _ng in bpy.data.node_groups:
    if not _ng.name.startswith("env:"):
        continue
    for _n in _ng.nodes:
        if _n.bl_idname == "ShaderNodeMapRange" and abs(_n.inputs["From Min"].default_value - 0.3) < 1e-6 \
                and abs(_n.inputs["From Max"].default_value - 0.7) < 1e-6:
            _n.inputs["To Min"].default_value = 0.0
            _n.inputs["To Max"].default_value = 0.0
            _nw += 1
print("GrappStar: cloud shadows off:", bool(_cs), "| wind nodes zeroed:", _nw)

if a.start is not None or a.end is not None:
    ranges = [(a.start if a.start is not None else sc.frame_start, a.end if a.end is not None else sc.frame_end)]
else:
    spec = "273-650,859-960" if CY else "305-400,600-640,870-940"     # short cut (Harry, deadline): launch to drills, line taut
    ranges = [tuple(int(x) for x in rr.split("-")) for rr in spec.split(",")]

if CY:
    a.engine = "cycles"
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
    sc.cycles.samples = {"preview": 24, "hd720": 48, "cy720": 16, "final": 96, "uhd": 128, "hero": 512}[a.preset]
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.use_denoising = True
    if CY:                          # M3 10-core GPU estimate: about 6-10 s a frame at 720p (16 samples + denoise);
        c = sc.cycles               # 24 samples would be about 9-14 s
        c.adaptive_threshold = 0.03
        c.adaptive_min_samples = 0
        try:
            c.denoiser = "OPENIMAGEDENOISE"
            c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
            c.denoising_use_gpu = True
        except Exception:
            pass
        c.max_bounces, c.diffuse_bounces, c.glossy_bounces = 5, 3, 2
        c.transmission_bounces, c.transparent_max_bounces, c.volume_bounces = 4, 8, 0
        c.sample_clamp_indirect = 5.0
        c.caustics_reflective = c.caustics_refractive = False
        r.use_persistent_data = True
else:
    r.engine = "BLENDER_EEVEE"
    ee = sc.eevee
    ee.taa_render_samples = {"preview": 12, "hd720": 10, "final": 32, "uhd": 48, "hero": 128}[a.preset]

a.out = a.out.rstrip("/") + ("_cy720/" if CY else "_v3/")     # v2: still grass, rendered fresh
os.makedirs(a.out, exist_ok=True)
for _f in os.listdir(a.out):                                  # empty placeholders from a stopped run: render them again
    if _f.startswith("f_") and _f.endswith(".png") and os.path.getsize(os.path.join(a.out, _f)) == 0:
        os.remove(os.path.join(a.out, _f))
r.filepath = os.path.join(a.out, "f_")
if hasattr(r.image_settings, "media_type"):
    r.image_settings.media_type = "IMAGE"
r.image_settings.file_format = "PNG"
r.use_overwrite = False                                       # resume: skip frames already on disk
r.use_placeholder = True
n = sum(b - s + 1 for s, b in ranges)
print(f"GrappStar render: {a.preset} {a.engine} {res[0]}x{res[1]} ranges {ranges} ({n} frames) -> {a.out}")
for s, b in ranges:
    sc.frame_start, sc.frame_end = s, b
    bpy.ops.render.render(animation=True)

if not a.no_video:                                            # join the PNGs (in frame order) into one MP4
    want = set()
    for s, b in ranges:
        want |= {"f_%04d.png" % f for f in range(s, b + 1)}
    files = sorted(f for f in os.listdir(a.out) if f in want)
    fps = r.fps
    vs = bpy.data.scenes.new("video")
    vs.render.resolution_x, vs.render.resolution_y, vs.render.resolution_percentage = res[0], res[1], 100
    vs.render.fps = fps
    vs.sequence_editor_create()
    if files:
        strip = vs.sequence_editor.strips.new_image("frames", os.path.join(a.out, files[0]), 1, 1) \
            if hasattr(vs.sequence_editor, "strips") else vs.sequence_editor.sequences.new_image("frames", os.path.join(a.out, files[0]), 1, 1)
        for f in files[1:]:
            strip.elements.append(f)
        if hasattr(strip, "frame_final_duration"):
            try:
                strip.frame_final_duration = len(files)
            except Exception:
                pass
        vs.frame_start, vs.frame_end = 1, len(files)
        if hasattr(vs.render.image_settings, "media_type"):
            vs.render.image_settings.media_type = "VIDEO"           # Blender 5: video output is its own media type
        vs.render.image_settings.file_format = "FFMPEG"
        vs.render.ffmpeg.format = "MPEG4"
        vs.render.ffmpeg.codec = "H264"
        vs.render.ffmpeg.constant_rate_factor = "MEDIUM"           # CRF 23: modest bitrate, real compression texture
        vs.render.filepath = os.path.join(a.out, "grappstar_mission.mp4")
        bpy.ops.render.render(animation=True, scene=vs.name)
        print("video:", vs.render.filepath, len(files), "frames")
