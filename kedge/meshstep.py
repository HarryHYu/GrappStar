from build123d import Solid, Shell, export_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.gp import gp_Pnt
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopoDS import TopoDS
def _clean(m):
    """merge vertices closer than 1e-3 and drop slivers: OCC sewing at 1e-3 otherwise makes invalid faces (foot auger v31)"""
    import trimesh, numpy as np
    t = m.copy(); t.merge_vertices(digits_vertex=3); t.update_faces(t.nondegenerate_faces(height=1e-3)); t.remove_unreferenced_vertices()
    trimesh.repair.fill_holes(t); t.fix_normals()
    return t if t.is_watertight and abs(t.volume - m.volume) < 1e-3 * max(abs(m.volume), 1) else m
def mesh_to_solid(m):
    m = _clean(m)
    sew = BRepBuilderAPI_Sewing(1e-3)
    V = m.vertices
    for f in m.faces:
        p = BRepBuilderAPI_MakePolygon(gp_Pnt(*V[f[0]]), gp_Pnt(*V[f[1]]), gp_Pnt(*V[f[2]]), True)
        fc = BRepBuilderAPI_MakeFace(p.Wire(), True)
        if fc.IsDone(): sew.Add(fc.Face())
    sew.Perform()
    sh = sew.SewedShape()
    ex = TopExp_Explorer(sh, TopAbs_SHELL)
    shell = TopoDS.Shell(ex.Current()) if hasattr(TopoDS, 'Shell') else ex.Current()
    ms = BRepBuilderAPI_MakeSolid(shell)
    return Solid(ms.Solid())
if __name__ == "__main__":
    import trimesh, time
    t = time.time(); m = trimesh.creation.icosphere(subdivisions=3, radius=5)
    S = mesh_to_solid(m); print(S.volume, S.is_valid, time.time() - t)
    export_step(S, 's.step')
