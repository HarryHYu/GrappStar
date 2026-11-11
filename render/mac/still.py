"""GrappStar: one photoreal still of the whole project (Cycles), from a custom camera.

Mac (after the video is done), in the grappstar_render folder:
  /Applications/Blender.app/Contents/MacOS/Blender -b grappstar_mission.blend -P still.py
  -> still_<name>.png (3840x2160, Cycles on the Apple GPU, 256 samples, denoised)
Options after "--": --view front|rear  --res 3840 2160  --samples 256  --frame 273  --cpu
"""
import sys, os, math, argparse
import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--view", default="rear")
ap.add_argument("--cam", default=None, help="x,y,z:tx,ty,tz:lens")
ap.add_argument("--res", nargs=2, type=int, default=[3840, 2160])
ap.add_argument("--samples", type=int, default=256)
ap.add_argument("--frame", type=int, default=273)        # dart cocked on the rail, everything at rest
ap.add_argument("--cpu", action="store_true")
ap.add_argument("--out", default=None)
a = ap.parse_args(argv)

VIEWS = {   # whole base station + dart on the rail, from about 2 m
    "rear":  "-1.35,1.55,0.95:0.12,-0.12,0.16:32",       # sun behind the camera: front lit
    "front": "1.75,-1.45,0.85:0.05,0.02,0.17:35",        # sun ahead: backlit, rim light
}
spec = a.cam or VIEWS[a.view]
p, t, lens = spec.split(":")
pos = Vector([float(v) for v in p.split(",")]); tgt = Vector([float(v) for v in t.split(",")])

sc = bpy.context.scene
sc.frame_set(a.frame)
cd = bpy.data.cameras.new("cam:still"); cd.lens = float(lens); cd.sensor_width = 36
cd.dof.use_dof = True; cd.dof.focus_distance = (tgt - pos).length; cd.dof.aperture_fstop = 8.0
co = bpy.data.objects.new("cam:still", cd); sc.collection.objects.link(co)
co.location = pos
co.rotation_euler = (tgt - pos).to_track_quat("-Z", "Y").to_euler()
for fc in (co.animation_data.action.fcurves if co.animation_data and co.animation_data.action else []):
    pass
sc.camera = co
for m in list(sc.timeline_markers):                        # markers bind cameras: drop them for the still
    m.camera = None

r = sc.render
r.engine = "CYCLES"
try:
    bpy.ops.preferences.addon_enable(module="cycles")
except Exception:
    pass
if not a.cpu:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
sc.cycles.samples = a.samples
sc.cycles.use_adaptive_sampling = True
sc.cycles.use_denoising = True
r.resolution_x, r.resolution_y, r.resolution_percentage = a.res[0], a.res[1], 100
r.use_motion_blur = False
r.use_compositing = False                                 # clean photo, no phone-footage look
r.film_transparent = False
if hasattr(r.image_settings, "media_type"):
    r.image_settings.media_type = "IMAGE"
r.image_settings.file_format = "PNG"
out = a.out or os.path.join(os.path.dirname(bpy.data.filepath), "still_%s.png" % a.view)
r.filepath = out
print("GrappStar still:", a.view, spec, a.res, a.samples, "samples ->", out)
bpy.ops.render.render(write_still=True)
print("saved", out)
