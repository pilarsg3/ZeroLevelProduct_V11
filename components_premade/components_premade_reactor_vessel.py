"""
Reactor vessel (RV) builder.

Constructs a hollow cylindrical vessel with a choice of bottom head geometry
and an optional top closure plate with arbitrary hole patterns.

The vessel shell always runs from z=0 to z=straight_h, so the top face is
exactly at z=straight_h — no bounding box lookup needed. The top plate
centroid is placed at z=straight_h + plate_thickness/2.

Example
-------
>>> vessel = create_reactor_vessel(
...     inner_d    = 4.72,
...     wall_t     = 0.04,
...     straight_h = 5.5,
...     bottom_head_type   = "ellipsoidal",
...     bottom_head_params = {"head_depth": 1.0},
...     top_plate_thickness = 0.1,
...     top_plate_hole_groups=[
...         dict(hole_diameter=0.52, layout="custom_angles",
...              angles_deg=[0.0, 90.0, 180.0, 270.0], placement_radius=1.7),
...     ],
... )
"""

from __future__ import annotations
from typing import Any

import cadquery as cq
import math


# ---------------------------------------------------------------------------
# Bottom head builders
# Each returns a solid with its rim in the XY plane at z=0,
# extending downward (z < 0).
# ---------------------------------------------------------------------------

def _head_flat(outer_d: float, plate_t: float, **_) -> cq.Workplane:
    """Flat circular end plate, z in [-plate_t, 0]."""
    return cq.Workplane("XY").circle(outer_d / 2).extrude(-plate_t)


def _head_hemispherical(outer_d: float, **_) -> cq.Workplane:
    """Hemispherical bottom head, z in [-outer_d/2, 0]."""
    r = outer_d / 2
    sphere = cq.Workplane("XY").sphere(r)
    cutter = cq.Workplane("XY").box(10 * outer_d, 10 * outer_d, 10 * outer_d).translate((0, 0, -5 * outer_d))
    return sphere.intersect(cutter)


def _head_ellipsoidal(outer_d: float, head_depth: float, n: int = 40, **_) -> cq.Workplane:
    """
    Ellipsoidal bottom head.
    head_depth is the dish depth (for 2:1 ellipsoidal use head_depth = outer_d/4).
    """
    r = outer_d / 2
    pts = [
        (r * math.cos(t), -head_depth * math.sin(t))
        for t in [i * (math.pi / 2) / n for i in range(n + 1)]
    ]
    prof = (
        cq.Workplane("XZ")
        .moveTo(*pts[0])
        .spline(pts[1:], includeCurrent=True)
        .lineTo(0, 0)
        .close()
    )
    return prof.revolve(360)


def _head_torispherical(
    outer_d: float,
    Rc: float | None = None,
    rk: float | None = None,
    n_crown: int = 80,
    n_knuckle: int = 40,
    **_,
) -> cq.Workplane:
    """
    Torispherical (flanged & dished) bottom head.
    Rc: crown radius (defaults to outer_d).
    rk: knuckle radius (defaults to 0.06 * outer_d).
    """
    r = outer_d / 2
    Rc = outer_d if Rc is None else Rc
    rk = 0.06 * outer_d if rk is None else rk

    if rk <= 0:          raise ValueError("rk must be > 0")
    if Rc <= r:          raise ValueError("Rc must be > outer_d/2")
    if Rc <= rk:         raise ValueError("Rc must be > rk")

    xk, zk = r - rk, 0.0
    d = Rc - rk
    rad = d * d - xk * xk
    if rad <= 0:
        raise ValueError("Infeasible Rc/rk for this outer_d (try larger Rc or smaller rk)")
    zc = math.sqrt(rad)

    dx, dz = xk - 0.0, zk - zc
    L = math.hypot(dx, dz)
    ux, uz = dx / L, dz / L
    xt = Rc * ux
    zt = zc + Rc * uz

    a0 = math.atan2(0.0 - zk, r - xk)
    a1 = math.atan2(zt - zk, xt - xk)
    if a1 > a0:
        a1 -= 2 * math.pi
    knuckle_pts = [
        (xk + rk * math.cos(a), zk + rk * math.sin(a))
        for a in [a0 + i * (a1 - a0) / n_knuckle for i in range(n_knuckle + 1)]
    ]

    b0 = math.atan2(zt - zc, xt)
    b1 = -math.pi / 2
    crown_pts = [
        (Rc * math.cos(b), zc + Rc * math.sin(b))
        for b in [b0 + i * (b1 - b0) / n_crown for i in range(1, n_crown + 1)]
    ]

    pts = knuckle_pts + crown_pts
    prof = (
        cq.Workplane("XZ")
        .moveTo(*pts[0])
        .spline(pts[1:], includeCurrent=True)
        .lineTo(0, 0)
        .close()
    )
    return prof.revolve(360)


_HEAD_BUILDERS = {
    "flat":          _head_flat,
    "hemispherical": _head_hemispherical,
    "ellipsoidal":   _head_ellipsoidal,
    "torispherical": _head_torispherical,
}


def _build_outer_head(outer_d: float, head_type: str, params: dict) -> cq.Workplane:
    """Build a bottom head solid at z=0 extending downward."""
    if head_type not in _HEAD_BUILDERS:
        raise ValueError(
            f"Unknown head type '{head_type}'. "
            f"Choose from: {list(_HEAD_BUILDERS)}"
        )
    return _HEAD_BUILDERS[head_type](outer_d, **params)


def _build_top_head(outer_d: float, head_type: str, params: dict, z0: float) -> cq.Workplane:
    """
    Build a top head whose rim lies at z=z0 and extends upward.
    Reuses the bottom head builders by mirroring about XY.
    """
    h = _build_outer_head(outer_d, head_type, params)
    return h.mirror("XY").translate((0, 0, z0))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def create_reactor_vessel(
    straight_h: float,
    *,
    inner_d: float | None = None,
    wall_t:  float | None = None,
    outer_d: float | None = None,
    bottom_head_type:   str | None = None,
    bottom_head_params: dict | None = None,
    top_head_type:      str | None = None,
    top_head_params:    dict | None = None,
    top_plate_thickness:   float | None = None,
    top_plate_hole_groups: list[dict[str, Any]] | None = None,
) -> cq.Workplane:
    """
    Build a reactor vessel with optional head geometries and top plate.

    The cylindrical shell always runs from z=0 to z=straight_h. The top face
    is therefore always exactly at z=straight_h, making top plate positioning
    exact without any bounding box calculation.

    Local origin / datum: vessel axis = local Z; z = 0 at the bottom of the
    straight shell (a bottom head, if any, extends below z = 0).

    Parameters
    ----------
    straight_h : float
        Height of the straight cylindrical section (unitless).
    inner_d : float, optional
        Inner diameter (unitless).
    wall_t : float, optional
        Wall thickness (unitless).
    outer_d : float, optional
        Outer diameter (unitless). At least two of {inner_d, wall_t, outer_d} must be supplied;
        the missing one is derived. If all three are given they must satisfy
        outer_d == inner_d + 2*wall_t.

    bottom_head_type : str, optional
        'flat' | 'hemispherical' | 'ellipsoidal' | 'torispherical'
        If None, the vessel has an open bottom.
    bottom_head_params : dict, optional
        Parameters forwarded to the bottom head builder:
          flat:          plate_t (defaults to wall_t)
          ellipsoidal:   head_depth (required)
          torispherical: Rc, rk (both optional, have defaults)
          hemispherical: no extra params needed

    top_head_type : str, optional
        Same options as bottom_head_type. If None, the vessel has an open top.
        Note: if top_plate_thickness is also given, the top head sits on top
        of the plate (head rim at z = straight_h + top_plate_thickness).
    top_head_params : dict, optional
        Same structure as bottom_head_params.

    top_plate_thickness : float, optional
        If given, a flat closure plate of this thickness is created and
        fused into the returned vessel solid. Its bottom face sits flush
        at z=straight_h; a top head, if present, sits on top of the plate.
    top_plate_hole_groups : list[dict], optional
        Passed directly to create_top_plate. See create_top_plate for the
        full hole_groups specification.

    Returns
    -------
    cq.Workplane
        Single fused solid: shell + heads, with the top plate fused in
        if top_plate_thickness is given.
    """
    if straight_h <= 0: raise ValueError("straight_h must be > 0")

    # Resolve inner_d / wall_t / outer_d: at least two must be given;
    # if all three are given they must satisfy outer_d == inner_d + 2*wall_t.
    given = sum(x is not None for x in (inner_d, wall_t, outer_d))
    if given < 2:
        raise ValueError("At least two of {inner_d, wall_t, outer_d} must be provided.")
    if outer_d is None:
        _outer_d: float = float(inner_d) + 2.0 * float(wall_t)  # type: ignore[arg-type]
        _inner_d: float = float(inner_d)                          # type: ignore[arg-type]
        _wall_t:  float = float(wall_t)                           # type: ignore[arg-type]
    elif inner_d is None:
        _wall_t  = float(wall_t)                                  # type: ignore[arg-type]
        _outer_d = float(outer_d)
        _inner_d = _outer_d - 2.0 * _wall_t
    elif wall_t is None:
        _inner_d = float(inner_d)
        _outer_d = float(outer_d)
        _wall_t  = (_outer_d - _inner_d) / 2.0
    else:
        _inner_d = float(inner_d)
        _outer_d = float(outer_d)
        _wall_t  = float(wall_t)
        if abs(_outer_d - (_inner_d + 2.0 * _wall_t)) > 1e-9:
            raise ValueError(
                f"Inconsistent dimensions: outer_d ({_outer_d}) != inner_d ({_inner_d}) + 2*wall_t ({_wall_t})."
            )
    outer_d, inner_d, wall_t = _outer_d, _inner_d, _wall_t

    if inner_d <= 0:        raise ValueError("inner_d must be > 0")
    if wall_t  <= 0:        raise ValueError("wall_t must be > 0")
    if outer_d <= inner_d:  raise ValueError("outer_d must be > inner_d")

    bottom_head_params = dict(bottom_head_params or {})
    top_head_params    = dict(top_head_params    or {})

    # defaults
    if bottom_head_type == "flat":
        bottom_head_params.setdefault("plate_t", wall_t)
    if top_head_type == "flat":
        top_head_params.setdefault("plate_t", wall_t)

    # validation
    if bottom_head_type == "ellipsoidal" and "head_depth" not in bottom_head_params:
        raise ValueError("bottom_head_type='ellipsoidal' requires head_depth in bottom_head_params")
    if top_head_type == "ellipsoidal" and "head_depth" not in top_head_params:
        raise ValueError("top_head_type='ellipsoidal' requires head_depth in top_head_params")

    # ------------------------------------------------------------------ #
    # 1.  Outer shell + heads                                             #
    # ------------------------------------------------------------------ #
    outer = cq.Workplane("XY").circle(outer_d / 2).extrude(straight_h)

    if bottom_head_type == "flat":
        t = float(bottom_head_params["plate_t"])
        outer = outer.union(cq.Workplane("XY").circle(outer_d / 2).extrude(-t))
    elif bottom_head_type is not None:
        outer = outer.union(_build_outer_head(outer_d, bottom_head_type, bottom_head_params))

    top_head_z = straight_h + (top_plate_thickness or 0.0)
    if top_head_type == "flat":
        t = float(top_head_params["plate_t"])
        outer = outer.union(
            cq.Workplane("XY").workplane(offset=top_head_z).circle(outer_d / 2).extrude(t)
        )
    elif top_head_type is not None:
        outer = outer.union(_build_top_head(outer_d, top_head_type, top_head_params, top_head_z))

    # ------------------------------------------------------------------ #
    # 2.  Inner bore (cutter)                                             #
    # ------------------------------------------------------------------ #
    inner = cq.Workplane("XY").circle(inner_d / 2).extrude(straight_h)

    if bottom_head_type == "ellipsoidal":
        hd = float(bottom_head_params["head_depth"])
        p = dict(bottom_head_params)
        p["head_depth"] = max(hd - wall_t, 1e-3)
        inner = inner.union(_build_outer_head(inner_d, "ellipsoidal", p))
    elif bottom_head_type not in (None, "flat"):
        inner = inner.union(_build_outer_head(inner_d, bottom_head_type, bottom_head_params))

    # The inner (cavity) dome MUST get the same lift as the outer dome
    # (top_head_z includes the top plate thickness). Placing it lower leaves
    # the dome crown solid: apex wall = wall_t + plate_t instead of wall_t.
    if top_head_type == "ellipsoidal":
        hd = float(top_head_params["head_depth"])
        p = dict(top_head_params)
        p["head_depth"] = max(hd - wall_t, 1e-3)
        inner = inner.union(_build_top_head(inner_d, "ellipsoidal", p, top_head_z))
    elif top_head_type not in (None, "flat"):
        inner = inner.union(_build_top_head(inner_d, top_head_type, top_head_params, top_head_z))

    vessel = outer.cut(inner).clean()

    # OCCT can leave the result as a compound of sub-shapes. Force a true
    # topological fusion so the STEP export contains exactly one solid per
    # component (required for DAGMC neutronics workflows).
    solids = vessel.solids().vals()
    fused = solids[0]
    for s in solids[1:]:
        fused = fused.fuse(s)  # type: ignore
    vessel = cq.Workplane().add(fused)









    # ------------------------------------------------------------------ #
    # 3.  Top plate                                                        #
    # ------------------------------------------------------------------ #
    top_plate = None
    if top_plate_thickness is not None:
        from components_premade.components_premade_top_plate import create_top_plate
        top_plate = create_top_plate(
            outer_d         = outer_d,
            thickness       = top_plate_thickness,
            center_coords   = (0.0, 0.0, straight_h + top_plate_thickness / 2.0),
            hole_groups     = top_plate_hole_groups,
        )

    # ------------------------------------------------------------------ #
    # 4.  Assemble                                                         #
    # ------------------------------------------------------------------ #
    if top_plate is not None:
        return vessel.union(top_plate).clean()
    return vessel


# ---------------------------------------------------------------------------
# Usage examples
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import cadquery as cq
    from ocp_vscode import show

    # ------------------------------------------------------------------
    # Example A: ellipsoidal bottom, no top head, flat top plate with
    # 4 IHX penetrations matching REACTOR_HX layout (r=1.7, 4-fold)
    # ------------------------------------------------------------------
    assembly_A = create_reactor_vessel(
        inner_d    = 4.72,
        wall_t     = 0.04,
        straight_h = 5.5,
        bottom_head_type   = "ellipsoidal",
        bottom_head_params = {"head_depth": 1.0},
        top_plate_thickness = 0.1,
        top_plate_hole_groups=[
            dict(
                hole_diameter    = 0.52,
                layout           = "custom_angles",
                angles_deg       = [0.0, 90.0, 180.0, 270.0],
                placement_radius = 1.7,
            ),
        ],
    )
    show(assembly_A)
    import time; time.sleep(2)
    # ------------------------------------------------------------------
    # Example B: torispherical bottom, flat top head, no top plate
    # ------------------------------------------------------------------
    assembly_B = create_reactor_vessel(
        inner_d    = 4.72,
        wall_t     = 0.04,
        straight_h = 5.5,
        bottom_head_type   = "torispherical",
        bottom_head_params = {"Rc": 4.72, "rk": 0.06 * 4.72},
        top_head_type      = "flat",
    )
    show(assembly_B)
    import time; time.sleep(2)
    # ------------------------------------------------------------------
    # Example C: ellipsoidal bottom, ellipsoidal top head, flat top plate
    # with a central penetration — plate sits on top of the dome.
    # ------------------------------------------------------------------
    assembly_C = create_reactor_vessel(
        inner_d    = 4.72,
        wall_t     = 0.04,
        straight_h = 5.5,
        bottom_head_type   = "ellipsoidal",
        bottom_head_params = {"head_depth": 1.0},
        top_head_type      = "ellipsoidal",
        top_head_params    = {"head_depth": 1.0},
        top_plate_thickness = 0.1,
        top_plate_hole_groups=[
            dict(
                hole_diameter    = 0.3,
                layout           = "custom_angles",
                angles_deg       = [0.0],
                placement_radius = 0.0,
            ),
        ],
    )
    show(assembly_C)