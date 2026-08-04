"""Small mesh and 2D geometry helpers."""
import math, numpy as np, trimesh
import trimesh.transformations as tf
from math import sin, cos, tan, radians, degrees


def ideg(a):
    ar = radians(a); return degrees(tan(ar) - ar)

def mesh_from_tris(tris):
    V = np.array([p for t in tris for p in t], dtype=float)
    F = np.arange(len(V)).reshape(-1, 3)
    m = trimesh.Trimesh(vertices=V, faces=F, process=True)
    m.merge_vertices(); m.remove_duplicate_faces() if hasattr(m, "remove_duplicate_faces") else None
    m.fix_normals()
    return m

def cyl(r, h, center=(0, 0, 0), axis="z", sections=96):
    c = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    if axis == "x":
        c.apply_transform(tf.rotation_matrix(math.pi / 2, [0, 1, 0]))
    elif axis == "y":
        c.apply_transform(tf.rotation_matrix(math.pi / 2, [1, 0, 0]))
    c.apply_translation(center)
    return c

def frustum(r0, r1, h, z0=0.0, sections=96):
    """Truncated cone (convex hull of two coaxial circles): radius r0 at z=z0, r1 at z=z0+h."""
    a = np.linspace(0, 2 * np.pi, sections, endpoint=False)
    p0 = np.c_[r0 * np.cos(a), r0 * np.sin(a), np.full(sections, z0)]
    p1 = np.c_[r1 * np.cos(a), r1 * np.sin(a), np.full(sections, z0 + h)]
    return trimesh.Trimesh(vertices=np.vstack([p0, p1])).convex_hull

def boxc(ext, center=(0, 0, 0)):
    b = trimesh.creation.box(extents=ext)
    b.apply_translation(center)
    return b

def sector2d(a0_deg, a1_deg, R, n=48):
    """A pie-slice polygon from the origin spanning [a0,a1] degrees at radius R."""
    import shapely.geometry as sg
    a = np.linspace(math.radians(a0_deg), math.radians(a1_deg), n)
    pts = [(0.0, 0.0)] + [(R*math.cos(t), R*math.sin(t)) for t in a]
    return sg.Polygon(pts)

def rbox(ext, center=(0, 0, 0), r=5.0):
    """A box with ROUNDED VERTICAL CORNERS (radius r), cleaner-looking casing than sharp edges,
    and trims a little corner material. Falls back to a sharp box if r is too big for the size."""
    import shapely.geometry as sg
    w, d, h = ext
    r = max(0.0, min(r, w/2 - 0.5, d/2 - 0.5))
    if r < 0.6:
        return boxc(ext, center)
    poly = sg.box(-w/2 + r, -d/2 + r, w/2 - r, d/2 - r).buffer(r, resolution=8, join_style=1)
    m = trimesh.creation.extrude_polygon(poly, h)
    m.apply_translation([center[0], center[1], center[2] - h/2])
    return m

def rot2(pts, phi):
    c, s = cos(radians(phi)), sin(radians(phi))
    return [(x * c - y * s, x * s + y * c) for (x, y) in pts]

def largest(m, _name=""):
    parts = m.split(only_watertight=False)
    if len(parts) <= 1: return m
    parts = sorted(parts, key=lambda p: p.volume, reverse=True)
    _dropped = sum(p.volume for p in parts[1:]) / 1000.0     # cm^3 discarded
    if _dropped > 1.0:                                        # a real sever (not a tessellation sliver), fail loudly
        print("  !! WARNING: %s split into %d bodies; _largest DROPPED %.1f cm^3, a sever/disconnect?"
              % (_name or "part", len(parts), _dropped))
    return parts[0]

def extrude_down(poly2d, h, top=0.0):
    """Extrude a shapely (Multi)Polygon downward to z∈[top−h, top]; concatenate parts."""
    import shapely.geometry as sg
    geoms = poly2d.geoms if poly2d.geom_type == 'MultiPolygon' else [poly2d]
    out = []
    for g in geoms:
        if g.is_empty or g.area < 0.1: continue
        m = trimesh.creation.extrude_polygon(g, h); m.apply_translation([0, 0, top - h]); out.append(m)
    return trimesh.util.concatenate(out) if out else None

def hex_poly(af):
    """Regular hexagon (shapely) with the given across-flats width, centred at the origin."""
    import shapely.geometry as sg
    r = af / math.sqrt(3.0)                          # corner radius from across-flats
    return sg.Polygon([(r * cos(radians(60*k + 30)), r * sin(radians(60*k + 30))) for k in range(6)])
