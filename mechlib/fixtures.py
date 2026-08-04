"""Pure fixture geometry."""
import trimesh

from mechlib.prim import boxc


def board_cradle(rect, fl, standoff=4.0, clr=0.4, wlen=9.0, wt=1.6, cap_over=2.0):
    """Per-component HOUSING for one PCB: at each of its 4 corners a standoff POST under the board
    (lifts the underside `standoff` mm off the floor so pins/solder don't short) + an L of two thin
    walls hugging that corner from OUTSIDE (locates the board in XY and captures its top edge). The
    brackets sit at corners only, so mid-edge connectors (USB, screw terminals, pin headers) stay
    clear, and the press-in lid holds the board down in Z. `fl` = interior floor z; `rect` = world
    (x0, y0, w, d, comp_h) from elec_board_rects(). Self-supporting: solid post, then walls rise on it."""
    x0, y0, w, d, _ = rect
    pcb_t = 1.6
    cap = fl + standoff + pcb_t + cap_over                  # capture-wall top, just over the PCB
    parts = []
    for ox in (-1, 1):
        for oy in (-1, 1):
            cx = x0 + (w if ox > 0 else 0)                  # this board corner (world XY)
            cy = y0 + (d if oy > 0 else 0)
            parts.append(boxc([5, 5, standoff], (cx - ox*2.5, cy - oy*2.5, fl + standoff/2.0)))     # post (under PCB)
            parts.append(boxc([wlen, wt, cap - fl], (cx - ox*wlen/2.0, cy + oy*(clr + wt/2.0), (fl + cap)/2.0)))  # wall ∥X edge
            parts.append(boxc([wt, wlen, cap - fl], (cx + ox*(clr + wt/2.0), cy - oy*wlen/2.0, (fl + cap)/2.0)))  # wall ∥Y edge
    return trimesh.boolean.union(parts)


def saddle(axis0, axis1, r, yi, rib_t, interior, rise=0.0):
    """Build a shell-trimmed cradle rib for a cylindrical part.

    This complements ``board_cradle`` for flat circuit boards.
    origin: mini-powerbank pickle_build.py:109
    """
    import numpy as np

    from .prim import seg_cylinder

    a0 = np.array(axis0, float); a1 = np.array(axis1, float)
    t = (yi - a0[1]) / (a1[1] - a0[1]); cz = a0[2] + t*(a1[2]-a0[2]); cx = a0[0] + t*(a1[0]-a0[0])
    top = cz + rise
    block = boxc((80, rib_t, top + 60), (cx, yi, top - (top+60)/2))
    block = trimesh.boolean.intersection([block, interior], engine='manifold')
    notch = seg_cylinder([a0[0], yi-rib_t, a0[2]],
                         [a1[0], yi+rib_t, a1[2]], 2 * (r+0.6))
    return trimesh.boolean.difference([block, notch], engine='manifold')
