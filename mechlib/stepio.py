"""Optional STEP assembly export through OpenCascade."""


def _shape(mesh):
    """Convert a triangle mesh into a sewn, same-domain-unified OCP shape."""
    from OCP.gp import gp_Pnt
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakePolygon,
                                    BRepBuilderAPI_MakeFace,
                                    BRepBuilderAPI_Sewing)
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain

    sew = BRepBuilderAPI_Sewing(1e-3)
    V = mesh.vertices
    for a, b, c in mesh.faces:
        tri = BRepBuilderAPI_MakePolygon(gp_Pnt(*V[a]), gp_Pnt(*V[b]), gp_Pnt(*V[c]), True)
        sew.Add(BRepBuilderAPI_MakeFace(tri.Wire()).Face())
    sew.Perform()
    u = ShapeUpgrade_UnifySameDomain(sew.SewedShape(), True, True, True)
    u.Build()
    return u.Shape()


def export_assembly(named_meshes, path):
    """Write already-positioned named trimesh parts as one STEP assembly.

    OCP is an optional dependency imported only while exporting.
    origin: dual-axis-turntable src/step_export.py:33
    """
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder
    from OCP.IFSelect import IFSelect_RetDone

    comp = TopoDS_Compound()
    bld = BRep_Builder()
    bld.MakeCompound(comp)
    for name, m in named_meshes.items():
        try:
            bld.Add(comp, _shape(m))
        except Exception as e:
            print("  STEP: skipped %s (%s)" % (name, e))
    w = STEPControl_Writer()
    w.Transfer(comp, STEPControl_AsIs)
    if w.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError("STEP write failed")
    return path
