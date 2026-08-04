"""Pure sweep geometry helpers."""
import math, numpy as np
from math import sin, cos, radians

from mechlib.prim import mesh_from_tris, rot2


def extrude_twist(base_outer, base_inner, zlist, phi_of, prof=None, base_ang=None):
    N = len(base_outer)
    if base_ang is None:
        base_ang = [math.atan2(y, x) for (x, y) in base_outer]
    Lo, Li = [], []
    for z in zlist:
        phi = phi_of(z)
        if prof is None:
            o = rot2(base_outer, phi)
        else:
            cp, sp = cos(radians(phi)), sin(radians(phi))
            o = []
            for i in range(N):
                r = prof(z, i); a = base_ang[i]
                x, y = r * cos(a), r * sin(a)
                o.append((x * cp - y * sp, x * sp + y * cp))
        Lo.append([(x, y, z) for (x, y) in o])
        if base_inner is not None:
            Li.append([(x, y, z) for (x, y) in rot2(base_inner, phi)])

    solid = base_inner is None
    tris = []
    for k in range(len(zlist) - 1):
        Ao, Bo = Lo[k], Lo[k + 1]
        for i in range(N):
            j = (i + 1) % N
            tris += [(Ao[i], Ao[j], Bo[i]), (Ao[j], Bo[j], Bo[i])]   # outer wall
        if not solid:
            Ai, Bi = Li[k], Li[k + 1]
            for i in range(N):
                j = (i + 1) % N
                tris += [(Ai[i], Bi[i], Ai[j]), (Ai[j], Bi[i], Bi[j])]  # inner wall

    bo, to = Lo[0], Lo[-1]
    if solid:                                    # caps = fan from centre
        cb, ct = (0, 0, zlist[0]), (0, 0, zlist[-1])
        for i in range(N):
            j = (i + 1) % N
            tris += [(cb, bo[j], bo[i]), (ct, to[i], to[j])]
    else:
        bi, ti = Li[0], Li[-1]
        for i in range(N):
            j = (i + 1) % N
            tris += [(bo[i], bi[i], bi[j]), (bo[i], bi[j], bo[j])]
            tris += [(to[i], ti[j], ti[i]), (to[i], to[j], ti[j])]
    return mesh_from_tris(tris)

def swept_keyed_bore(bore_poly, free_angle, steps=28):
    """Variant B manual override: sweep the keyed profile through `free_angle` into a
    fan-shaped bore (angular lost motion). Drive contact stays flat-to-flat at the fan's
    end walls; the mid-travel arc is unloaded free play."""
    import shapely.affinity as sa
    polys = [sa.rotate(bore_poly, a, origin=(0, 0), use_radians=False)
             for a in np.linspace(0.0, free_angle, steps)]
    swept = polys[0]
    for p in polys[1:]:
        swept = swept.union(p)
    return swept.buffer(0)


def ring_pts(poly, n, z):
    """Resample a polygon boundary to evenly spaced 3D points at height Z.

    origin: dual-axis-turntable src/build.py:193
    """
    L = poly.exterior.length
    pts = [poly.exterior.interpolate(i / n * L) for i in range(n)]
    return np.array([[p.x, p.y, z] for p in pts])


def loft(rings):
    """Build a solid between equal-count point rings with centroid-fan caps.

    origin: dual-axis-turntable src/build.py:200
    """
    import trimesh

    n = len(rings[0])
    # Minimal-twist correspondence: ring boundaries can start at arbitrary points
    # (e.g. rotated polygons), so re-index each ring to the cyclic shift closest to
    # the previous ring. Already-aligned rings keep shift 0 and pass through as-is.
    aligned = [np.asarray(rings[0])]
    for ring in rings[1:]:
        current = np.asarray(ring)
        previous = aligned[-1]
        shift = min(
            range(n),
            key=lambda i: np.sum((previous - np.roll(current, -i, axis=0)) ** 2),
        )
        aligned.append(current if shift == 0 else np.roll(current, -shift, axis=0))
    rings = aligned
    V = np.vstack(rings).tolist()
    F = []
    for k in range(len(rings) - 1):
        a, b = k * n, (k + 1) * n
        for j in range(n):
            j2 = (j + 1) % n
            F.append([a + j, a + j2, b + j2]); F.append([a + j, b + j2, b + j])
    c0 = len(V); V.append(np.vstack(rings[0]).mean(0).tolist())
    for j in range(n):
        F.append([c0, (j + 1) % n, j])
    base = (len(rings) - 1) * n
    c1 = len(V); V.append(np.vstack(rings[-1]).mean(0).tolist())
    for j in range(n):
        F.append([c1, base + j, base + (j + 1) % n])
    m = trimesh.Trimesh(vertices=np.array(V), faces=np.array(F), process=True)
    m.fix_normals()
    return m
