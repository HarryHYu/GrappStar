"""HD still of the whole GrappStar kit on a dirty workshop table (no grass): base station on its plywood board, KEDGE
v35 dart on the rail, electronics beside it, carpentry-shop HDRI light. Cycles.

  render/.venv/bin/python render/table_still.py [--view a|b|dart] [--res 1600 900] [--samples 48] [--out file.png]
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bpy
from mathutils import Vector

ap = argparse.ArgumentParser()
ap.add_argument("--view", default="a")
ap.add_argument("--res", nargs=2, type=int, default=[1600, 900])
ap.add_argument("--samples", type=int, default=48)
ap.add_argument("--out", default=None)
ap.add_argument("--gpu", action="store_true")
a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:])

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
import models
models.build_models(sc)

T = os.path.join(HERE, "assets", "cache", "table")
TOP = -models.BOARD_SIZE[2]                                   # table top under the plywood board


def img(nt, path, non_color=False, vec=None):
    n = nt.nodes.new("ShaderNodeTexImage")
    n.image = bpy.data.images.load(path)
    if non_color:
        n.image.colorspace_settings.name = "Non-Color"
    if vec is not None:
        nt.links.new(vec, n.inputs["Vector"])
    return n


# ---- dirty table: old planks, grime, scuffs, uneven sheen ----
m = bpy.data.materials.new("table:top"); m.use_nodes = True
nt = m.node_tree; N, L = nt.nodes, nt.links
bsdf = N["Principled BSDF"]
tc = N.new("ShaderNodeTexCoord"); mp = N.new("ShaderNodeMapping")
mp.inputs["Scale"].default_value = (0.55, 0.55, 0.55)
L.new(tc.outputs["Object"], mp.inputs["Vector"])
dif = img(nt, os.path.join(T, "diff.jpg"), vec=mp.outputs[0])
rgh = img(nt, os.path.join(T, "rough.jpg"), True, mp.outputs[0])
nor = img(nt, os.path.join(T, "nor.jpg"), True, mp.outputs[0])
grime = N.new("ShaderNodeTexNoise"); grime.inputs["Scale"].default_value = 2.5; grime.inputs["Detail"].default_value = 8
L.new(tc.outputs["Object"], grime.inputs["Vector"])
gramp = N.new("ShaderNodeMapRange"); gramp.inputs["From Min"].default_value = 0.45; gramp.inputs["From Max"].default_value = 0.7
L.new(grime.outputs["Fac"], gramp.inputs["Value"])
mix = N.new("ShaderNodeMix"); mix.data_type = "RGBA"; mix.blend_type = "MULTIPLY"
L.new(gramp.outputs["Result"], mix.inputs["Factor"]); L.new(dif.outputs["Color"], mix.inputs["A"])
mix.inputs["B"].default_value = (0.32, 0.27, 0.22, 1)
L.new(mix.outputs["Result"], bsdf.inputs["Base Color"])
smudge = N.new("ShaderNodeTexNoise"); smudge.inputs["Scale"].default_value = 6.0; smudge.inputs["Detail"].default_value = 4
L.new(tc.outputs["Object"], smudge.inputs["Vector"])
rmix = N.new("ShaderNodeMath"); rmix.operation = "MULTIPLY_ADD"
L.new(rgh.outputs["Color"], rmix.inputs[0]); rmix.inputs[1].default_value = 0.7
L.new(smudge.outputs["Fac"], rmix.inputs[2])
rclamp = N.new("ShaderNodeClamp"); L.new(rmix.outputs[0], rclamp.inputs["Value"])
rclamp.inputs["Min"].default_value = 0.45
L.new(rclamp.outputs[0], bsdf.inputs["Roughness"])
nm = N.new("ShaderNodeNormalMap"); nm.inputs["Strength"].default_value = 1.2
L.new(nor.outputs["Color"], nm.inputs["Color"]); L.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
bsdf.inputs["Specular IOR Level"].default_value = 0.35

bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.1, 0.0, TOP - 0.025))
top = bpy.context.active_object; top.name = "table:top"
top.scale = (2.4, 1.3, 0.05); top.data.materials.append(m)
bpy.ops.object.transform_apply(scale=True)
bev = top.modifiers.new("bevel", "BEVEL"); bev.width = 0.004; bev.segments = 3

# sawdust, screws and offcuts would go here; keep it to dust specks via the grime noise (time)

# ---- workshop wall far behind, out of focus ----
wm = bpy.data.materials.new("table:wall"); wm.use_nodes = True
wn = wm.node_tree; wb = wn.nodes["Principled BSDF"]
wtc = wn.nodes.new("ShaderNodeTexCoord"); wmp = wn.nodes.new("ShaderNodeMapping"); wmp.inputs["Scale"].default_value = (0.4,) * 3
wn.links.new(wtc.outputs["Object"], wmp.inputs["Vector"])
wi = img(wn, os.path.join(T, "wall.jpg"), vec=wmp.outputs[0]); wn.links.new(wi.outputs["Color"], wb.inputs["Base Color"])
wb.inputs["Roughness"].default_value = 0.9

# ---- light: carpentry shop HDRI + a soft window key ----
w = bpy.data.worlds.new("shop"); sc.world = w; w.use_nodes = True
env = w.node_tree.nodes.new("ShaderNodeTexEnvironment"); env.image = bpy.data.images.load(os.path.join(T, "shop.hdr"))
bg = w.node_tree.nodes["Background"]; bg.inputs["Strength"].default_value = 1.0
mpw = w.node_tree.nodes.new("ShaderNodeMapping"); tcw = w.node_tree.nodes.new("ShaderNodeTexCoord")
mpw.inputs["Rotation"].default_value = (0, 0, math.radians(35))
w.node_tree.links.new(tcw.outputs["Generated"], mpw.inputs["Vector"]); w.node_tree.links.new(mpw.outputs[0], env.inputs["Vector"])
w.node_tree.links.new(env.outputs["Color"], bg.inputs["Color"])
ld = bpy.data.lights.new("window", "AREA"); ld.energy = 220; ld.size = 1.6; ld.color = (1.0, 0.95, 0.88)
lo = bpy.data.objects.new("window", ld); sc.collection.objects.link(lo)
lo.location = (-1.2, -1.6, 1.6); lo.rotation_euler = (Vector((0.1, 0, 0.1)) - lo.location).to_track_quat("-Z", "Y").to_euler()

# ---- camera ----
VIEWS = {"a": ((1.25, -1.05, 0.62), (0.04, 0.02, 0.13), 38),
         "b": ((-1.1, 1.15, 0.7), (0.1, -0.08, 0.13), 38),
         "dart": ((0.55, -0.55, 0.42), (0.12, 0.08, 0.24), 55)}
p, t, lens = VIEWS[a.view]
cd = bpy.data.cameras.new("cam"); cd.lens = lens
cd.dof.use_dof = True; cd.dof.focus_distance = (Vector(t) - Vector(p)).length; cd.dof.aperture_fstop = 4.0
co = bpy.data.objects.new("cam", cd); sc.collection.objects.link(co); co.location = p
co.rotation_euler = (Vector(t) - Vector(p)).to_track_quat("-Z", "Y").to_euler(); sc.camera = co

r = sc.render
r.engine = "CYCLES"
if a.gpu:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"; prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
sc.cycles.samples = a.samples; sc.cycles.use_adaptive_sampling = True; sc.cycles.use_denoising = True
sc.cycles.max_bounces = 6
r.resolution_x, r.resolution_y, r.resolution_percentage = a.res[0], a.res[1], 100
sc.view_settings.view_transform = "AgX"
try:
    sc.view_settings.look = "AgX - Medium High Contrast"
except Exception:
    pass
r.image_settings.file_format = "PNG"
r.filepath = a.out or os.path.join(HERE, "out", "table_%s.png" % a.view)
bpy.ops.render.render(write_still=True)
print("saved", r.filepath)
