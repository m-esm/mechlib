"""Mesh conversion, CSG, probing, and polygon extrusion helpers."""

import itertools

import manifold3d as _m3
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix as R

from .prim import boxc


def to_manifold(mesh):
    """Convert a trimesh mesh to a manifold3d solid.

    origin: parviz src/geo.py:274
    """
    return _m3.Manifold(_m3.Mesh(
        vert_properties=np.array(mesh.vertices, dtype=np.float32, order="C", subok=False),
        tri_verts=np.array(mesh.faces, dtype=np.uint32, order="C", subok=False)))


def from_manifold(man):
    """Convert a manifold3d solid to a trimesh mesh.

    origin: parviz src/geo.py:287
    """
    out = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(out.vert_properties)[:, :3],
                           faces=np.asarray(out.tri_verts), process=False)


def sub(a, b):
    """Subtract mesh ``b`` from mesh ``a``.

    origin: parviz src/geo.py:293
    """
    return from_manifold(to_manifold(a) - to_manifold(b))


def uni(parts):
    """Union a sequence of meshes through manifold3d.

    origin: parviz src/geo.py:297
    """
    m = to_manifold(parts[0])
    for p in parts[1:]:
        m = m + to_manifold(p)
    return from_manifold(m)


def inter(a, b):
    """Intersect two meshes through manifold3d.

    origin: parviz src/geo.py:304
    """
    return from_manifold(to_manifold(a) ^ to_manifold(b))


def export_stl(mesh, path):
    """Write STL after float32-aware watertight repair and geometry guards.

    origin: parviz src/geo.py:46
    """
    chk = trimesh.Trimesh(vertices=np.asarray(mesh.vertices, dtype=np.float32)
                          .astype(np.float64),
                          faces=mesh.faces.copy(), process=True)
    if not chk.is_watertight:
        mg = _m3.Mesh(
            vert_properties=np.asarray(chk.vertices, dtype=np.float32),
            tri_verts=np.asarray(chk.faces, dtype=np.uint32))
        out = _m3.Manifold(mesh=mg).to_mesh()
        rep = trimesh.Trimesh(vertices=np.asarray(out.vert_properties)[:, :3],
                              faces=np.asarray(out.tri_verts))
        dvol = abs(rep.volume - mesh.volume) / max(abs(mesh.volume), 1e-9)
        dbox = float(np.abs(np.asarray(rep.bounds) - np.asarray(mesh.bounds)).max())
        assert dvol < 1e-3 and dbox < 0.01, (
            "export repair changed %s: dvol %.4f%%, dbbox %.4f mm"
            % (path, 100 * dvol, dbox))
        assert rep.is_watertight, "export mesh NOT WATERTIGHT after repair: %s" % path
        mesh = rep
    mesh.export(path)


def inflate(m, gap=0.6):
    """Move vertices outward along normals by a uniform clearance.

    origin: finnish-windows src/build_combined.py:139
    """
    g=m.copy(); g.vertices=g.vertices+g.vertex_normals*gap; g.merge_vertices(); return g


def bbox_overlap(a, b):
    """Return whether two mesh axis-aligned bounds overlap.

    origin: parviz src/assembly_check.py:150
    """
    amin, amax = a.bounds
    bmin, bmax = b.bounds
    return bool((amax >= bmin).all() and (bmax >= amin).all())


def overlap_volume(a, b):
    """Return boolean-intersection volume, or zero for disjoint bounds.

    origin: parviz src/assembly_check.py:156
    """
    if not bbox_overlap(a, b):
        return 0.0

    def _man(t):
        return _m3.Manifold(_m3.Mesh(
            vert_properties=np.asarray(t.vertices, dtype=np.float64),
            tri_verts=np.asarray(t.faces, dtype=np.uint32)))
    return abs(float((_man(a) ^ _man(b)).volume()))


def inside(mesh, pts):
    """Return whether all probe points lie inside a solid.

    origin: parviz src/checks.py:64
    """
    return bool(mesh.contains(np.atleast_2d(np.asarray(pts, float))).all())


def clear(mesh, pts):
    """Return whether no probe point lies inside a solid.

    origin: parviz src/checks.py:69
    """
    return not mesh.contains(np.atleast_2d(np.asarray(pts, float))).any()


def bore_pierces(mesh, start, direction, length, n=12):
    """Probe whether a bore stays open along its real axis.

    origin: parviz src/checks.py:74
    """
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    pts = np.asarray(start, float) + np.outer(np.linspace(0.0, length, n), d)
    return clear(mesh, pts)


def void_cube(mesh, pt, s=0.5):
    """Probe a small cube that should be void using manifold intersection.

    origin: parviz src/checks.py:82
    """
    c = boxc((s, s, s))
    c.apply_translation(np.asarray(pt, float))
    try:
        return inter(mesh, c).volume < 1e-9
    except Exception:
        return False


def solid_cube(mesh, pt, s=0.25):
    """Probe a small cube that should be mostly solid.

    origin: parviz src/checks.py:94
    """
    c = boxc((s, s, s))
    c.apply_translation(np.asarray(pt, float))
    try:
        return inter(mesh, c).volume > 0.8 * s ** 3
    except Exception:
        return False


def self_thickness(mesh, n, seed=0):
    """Return first-percentile and minimum ray thickness plus its location.

    The source sampling seed was 0 and is exposed here as ``seed``.
    origin: parviz src/wallcheck.py:229
    """
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=seed)
    th = trimesh.proximity.thickness(mesh, pts, method="ray")
    ok = np.isfinite(th) & (th > 1e-3)
    th, pts = th[ok], pts[ok]
    if th.size == 0:
        return None, None, None
    i = int(np.argmin(th))
    return float(np.percentile(th, 1)), float(th[i]), pts[i]


def decimate(m, pitch):
    """Apply cheap vertex-clustering decimation on a regular grid.

    origin: parviz src/refparts.py:85
    """
    v = np.round(np.asarray(m.vertices) / pitch) * pitch
    d = trimesh.Trimesh(vertices=v, faces=m.faces, process=True)
    d.update_faces(d.nondegenerate_faces())
    d.remove_unreferenced_vertices()
    return d


def cube_rotations():
    """Return the 24 proper rotations of a cube.

    origin: parviz src/refparts.py:112
    """
    out = []
    for perm in itertools.permutations(range(3)):
        for sx in (1, -1):
            for sy in (1, -1):
                for sz in (1, -1):
                    M = np.zeros((3, 3))
                    s = (sx, sy, sz)
                    for i, p in enumerate(perm):
                        M[i, p] = s[i]
                    if round(np.linalg.det(M)) == 1:
                        out.append(M)
    return out


_ROT24 = cube_rotations()


def fit_transform(real, placeholder):
    """Fit a real mesh rigidly to an oriented placeholder by sampled surfaces.

    SciPy is imported only when this optional helper is called.
    origin: parviz src/refparts.py:181
    """
    from scipy.spatial import cKDTree
    Po = np.asarray(placeholder.bounding_box_oriented.primitive.transform)
    Rw = Po[:3, :3]
    Pc = np.asarray(placeholder.centroid)
    Rc = np.asarray(real.centroid)
    ps = placeholder.sample(500) - Pc
    rs = real.sample(500) - Rc
    ps_obb = ps @ Rw
    tree = cKDTree(ps_obb)
    best, bestM = None, None
    for M in _ROT24:
        d, _ = tree.query(rs @ M.T)
        c = float(d.mean())
        if best is None or c < best:
            best, bestM = c, M
    Rworld = Rw @ bestM
    T = np.eye(4)
    T[:3, :3] = Rworld
    T[:3, 3] = Pc - Rworld @ Rc
    return T


def orient(m, normal):
    """Rotate a positive-Z-aligned mesh so positive Z points along a normal.

    origin: parviz src/geo.py:221
    """
    n = np.asarray(normal, float); n /= np.linalg.norm(n)
    z = np.array([0, 0, 1.0]); v = np.cross(z, n); s = np.linalg.norm(v)
    if s > 1e-6:
        m.apply_transform(R(np.arctan2(s, np.dot(z, n)), v / s))
    elif np.dot(z, n) < 0:
        m.apply_transform(R(np.pi, [1, 0, 0]))
    return m


def largest_poly(g):
    """Pick the largest polygon from a Shapely geometry.

    origin: finnish-doors src/projects/klonk/housings.py:332
    """
    if g is None or g.is_empty:
        return g
    if g.geom_type == "Polygon":
        return g
    if g.geom_type == "MultiPolygon":
        return max(g.geoms, key=lambda p: p.area)
    g2 = g.buffer(0)
    if g2.is_empty:
        return g2
    if g2.geom_type == "Polygon":
        return g2
    if g2.geom_type == "MultiPolygon":
        return max(g2.geoms, key=lambda p: p.area)
    return g2


def extrude_poly_z(poly2d, z0, z1):
    """Extrude a Polygon or MultiPolygon between two world Z planes.

    origin: finnish-doors src/projects/klonk/housings.py:435
    """
    h = float(z1 - z0)
    if h <= 1e-9 or poly2d is None or poly2d.is_empty:
        return None
    if poly2d.geom_type == "MultiPolygon":
        parts = []
        for g in poly2d.geoms:
            if g.is_empty or g.area < 1e-4:
                continue
            m = trimesh.creation.extrude_polygon(g, h)
            m.apply_translation([0.0, 0.0, z0])
            parts.append(m)
        if not parts:
            return None
        if len(parts) == 1:
            return parts[0]
        return trimesh.util.concatenate(parts)
    m = trimesh.creation.extrude_polygon(poly2d, h)
    m.apply_translation([0.0, 0.0, z0])
    return m


def bbox_gap(a, b):
    """Return the lower-bound gap between two axis-aligned bounds."""
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    gap = np.maximum(lo - hi, 0)
    return float(np.linalg.norm(gap))


def min_distance(a, b, n=800):
    """Approximate unsigned surface distance from sampled points.

    origin: torque-lever assembly_check.py:80
    """
    pts, _ = trimesh.sample.sample_surface(a, n)
    d = trimesh.proximity.ProximityQuery(b).on_surface(pts)[1]
    return float(d.min())


def audit(parts, clearance, allow, label="", aliases={}):
    """Audit all part pairs for overlap and sub-clearance proximity.

    ``allow`` contains frozenset name pairs. ``aliases`` maps split part names
    back to their logical parents.
    origin: torque-lever assembly_check.py:95
    """
    failures, warnings = [], []
    names = sorted(parts)
    for na, nb in itertools.combinations(names, 2):
        a, b = parts[na], parts[nb]
        if bbox_gap(a, b) > clearance:
            continue
        try:
            vol = overlap_volume(a, b)
        except Exception:
            vol = None
        na, nb = aliases.get(na, na), aliases.get(nb, nb)
        if na == nb:
            continue
        pair = frozenset((na, nb))
        if vol is None:
            warnings.append("  ?  %s vs %s: boolean failed (non-volume mesh), "
                            "fix watertightness and re-run" % (na, nb))
            continue
        if vol > 1e-3:
            line = "%s vs %s: OVERLAP %.2f mm^3%s" % (na, nb, vol, label)
            if pair in allow:
                warnings.append("  ~  %s (whitelisted)" % line)
            else:
                failures.append("  X  %s" % line)
            continue
        d = min_distance(a, b)
        if d < clearance:
            warnings.append("  !  %s vs %s: TIGHT %.2f mm (< %s)%s"
                            % (na, nb, d, clearance, label))
    return failures, warnings


def approach_clear(mesh, mouth, direction, max_dist=50.0):
    """Measure free travel toward an opening by marching along an approach.

    origin: jumper-wire-sockets src/checks.py:624
    """
    mx, my, mz = mouth
    dx, dy, dz = direction
    n = float(np.linalg.norm([dx, dy, dz])) or 1.0
    dx, dy, dz = dx / n, dy / n, dz / n
    lo, hi = mesh.bounds
    for dist in np.linspace(0.5, max_dist, 100):
        px = mx + dx * dist
        py = my + dy * dist
        pz = mz + dz * dist
        outside = (
            px < lo[0] - 0.2
            or px > hi[0] + 0.2
            or py < lo[1] - 0.2
            or py > hi[1] + 0.2
            or pz < lo[2] - 0.2
            or pz > hi[2] + 0.2
        )
        if outside:
            return float(max_dist)
        if mesh.contains([[px, py, pz]])[0]:
            return max(0.0, float(dist) - 0.5)
    return float(max_dist)


def slicer_area(polys, infill, line_w=0.42, perimeters=3):
    """Predict per-layer extrusion area from perimeter band plus infill core.

    origin: tripod lighten_legs.py:45
    """
    band = line_w * perimeters
    tot = 0.0
    for poly in polys:
        core = poly.buffer(-band)
        ca = core.area if not core.is_empty else 0.0
        tot += (poly.area - ca) + infill * ca
    return tot


def extrude_snapped(poly2d, z0, z1):
    """Snap Shapely vertices to 1e-6 before extrusion into mesh volumes.

    Prefer this for tangent or near-coincident vertices that can triangulate as
    non-volumes. Prefer ``extrude_poly_z`` for its Polygon/MultiPolygon mesh API
    when input geometry is already clean. Returns a list of volumes.
    origin: torque-lever build.py:117
    """
    import shapely

    poly2d = shapely.set_precision(poly2d, 1e-6)
    geoms = poly2d.geoms if poly2d.geom_type == "MultiPolygon" else [poly2d]
    out = []
    for g in geoms:
        m = trimesh.creation.extrude_polygon(g, z1 - z0)
        m.apply_translation([0, 0, z0])
        out.append(m)
    return out
