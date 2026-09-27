#!/usr/bin/env python3
"""
Parametric generator for the 3D-printable parts of the rover.

Only needs Python 3 + numpy + PyYAML (both ship with ROS 2). No CAD program is
required: every part is a union of 2.5D extrusions (polygon + holes), which is
triangulated with ear clipping and written as a binary STL.

Dimensions come from src/rover_description/config/dimensions.yaml, the
same file the URDF uses, so the simulated robot always matches the printed one.

Usage:
    python3 generate_parts.py                  # both robots
    python3 generate_parts.py --robot rover4   # only the 4WD Ackermann robot
    python3 generate_parts.py --check          # also verifies every mesh is watertight

Outputs (millimetres, already in print orientation, flat side down):
    stl/*.stl                               -> rover (2WD) parts: slice and print these
    stl/rover4/*.stl                        -> rover4 (4WD Ackermann) parts
    ../src/rover_description/meshes          -> copies used as visuals in the simulator
"""
import argparse
import math
import shutil
import struct
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
DIMS_FILE = HERE.parent / 'src/rover_description/config/dimensions.yaml'
DIMS_FILE_ROVER4 = HERE.parent / 'src/rover_description/config/dimensions_rover4.yaml'
STL_DIR = HERE / 'stl'
MESH_DIR = HERE.parent / 'src/rover_description/meshes'

# Parts that are also used as visual meshes in the URDF
VISUAL_PARTS = ['base_plate', 'top_plate', 'wheel_hub', 'tire_tpu', 'motor_bracket',
                'caster_spacer', 'lidar_riser', 'camera_riser']

M3 = 3.4        # clearance hole for M3 screws
M25 = 2.9       # clearance hole for M2.5 screws (Raspberry Pi)
QUARTER = 6.6   # clearance hole for 1/4"-20 camera tripod screw


# ---------------------------------------------------------------------------
# 2D shapes (lists of (x, y) tuples, mm)
# ---------------------------------------------------------------------------
def circle(cx, cy, r, n=None):
    n = n or max(16, int(2 * math.pi * r / 1.2))
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def rounded_rect(cx, cy, w, h, r, n_corner=8):
    r = min(r, w / 2 - 1e-3, h / 2 - 1e-3)
    pts = []
    corners = [(cx + w / 2 - r, cy + h / 2 - r, 0.0), (cx - w / 2 + r, cy + h / 2 - r, 90.0),
               (cx - w / 2 + r, cy - h / 2 + r, 180.0), (cx + w / 2 - r, cy - h / 2 + r, 270.0)]
    for (ox, oy, a0) in corners:
        for i in range(n_corner + 1):
            a = math.radians(a0 + 90.0 * i / n_corner)
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return pts


def rect(cx, cy, w, h):
    return [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2),
            (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]


def slot(cx, cy, length, width, angle_deg=0.0, n=10):
    """Stadium-shaped slot, `length` measured tip to tip."""
    r = width / 2
    half = max(length / 2 - r, 0.0)
    pts = []
    for sign, a0 in ((1, -90.0), (-1, 90.0)):
        for i in range(n + 1):
            a = math.radians(a0 + 180.0 * i / n)
            pts.append((sign * half + r * math.cos(a), r * math.sin(a)))
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


def d_shape(cx, cy, dia, flat_depth, n=32):
    """D-shaft hole: circle with a flat on the +y side."""
    r = dia / 2
    y_flat = r - flat_depth
    a_flat = math.asin(y_flat / r)
    a0, a1 = math.pi - a_flat, 2 * math.pi + a_flat
    pts = [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n))
           for i in range(n + 1)]
    return pts


# ---------------------------------------------------------------------------
# Triangulation: ear clipping with hole bridging
# ---------------------------------------------------------------------------
def _area(poly):
    a = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        a += x1 * y2 - x2 * y1
    return a / 2


def _point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def _segments_cross(p, q, a, b):
    """Vectorised proper intersection of segment pq with segments a[i]b[i]."""
    def orient(p1, p2, p3):
        return (p2[..., 0] - p1[..., 0]) * (p3[..., 1] - p1[..., 1]) - \
               (p2[..., 1] - p1[..., 1]) * (p3[..., 0] - p1[..., 0])
    p = np.broadcast_to(p, a.shape)
    q = np.broadcast_to(q, a.shape)
    d1, d2 = orient(p, q, a), orient(p, q, b)
    d3, d4 = orient(a, b, p), orient(a, b, q)
    eps = 1e-9
    return (((d1 > eps) & (d2 < -eps)) | ((d1 < -eps) & (d2 > eps))) & \
           (((d3 > eps) & (d4 < -eps)) | ((d3 < -eps) & (d4 > eps)))


def _edges(poly):
    a = np.asarray(poly, dtype=float)
    return a, np.roll(a, -1, axis=0)


def _bridge(poly, hole, others):
    """Merge `hole` (CW) into `poly` (CCW) through the shortest visible bridge."""
    mi = max(range(len(hole)), key=lambda i: (hole[i][0], hole[i][1]))
    m = np.array(hole[mi])
    all_edges = [_edges(poly), _edges(hole)] + [_edges(h) for h in others]
    ea = np.concatenate([e[0] for e in all_edges])
    eb = np.concatenate([e[1] for e in all_edges])
    order = sorted(range(len(poly)), key=lambda i: (poly[i][0] - m[0]) ** 2 + (poly[i][1] - m[1]) ** 2)
    for pi in order:
        p = np.array(poly[pi])
        if _segments_cross(m, p, ea, eb).any():
            continue
        # reject bridges that graze a vertex (collinear touching is not a proper crossing)
        d = p - m
        rel = ea - m
        tpar = rel @ d / (d @ d)
        dist = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0]) / math.sqrt(d @ d)
        if ((tpar > 1e-9) & (tpar < 1 - 1e-9) & (dist < 1e-7)).any():
            continue
        mid = tuple((m + p) / 2)
        if not _point_in_poly(mid, poly) or _point_in_poly(mid, hole) or \
                any(_point_in_poly(mid, h) for h in others):
            continue
        return poly[:pi + 1] + hole[mi:] + hole[:mi + 1] + poly[pi:]
    raise RuntimeError('could not bridge hole')


def triangulate(outer, holes=()):
    """Triangulate a polygon with holes. Aligned holes can create degenerate (collinear) bridges
    that stall ear clipping; in that case retry on a slightly rotated copy and rotate back."""
    for deg in (0.0, 0.37, 1.13, 2.71):
        c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        rot = lambda poly: [(x * c - y * s_, x * s_ + y * c) for x, y in poly]
        try:
            pts, tris = _triangulate(rot(outer), [rot(h) for h in holes])
        except RuntimeError:
            continue
        back = np.column_stack([pts[:, 0] * c + pts[:, 1] * s_, -pts[:, 0] * s_ + pts[:, 1] * c])
        return back, tris
    raise RuntimeError('triangulation failed')


def _triangulate(outer, holes=()):
    outer = list(outer) if _area(outer) > 0 else list(reversed(outer))
    holes = [list(h) if _area(h) < 0 else list(reversed(h)) for h in holes]
    poly = outer
    remaining = sorted(holes, key=lambda h: -max(p[0] for p in h))
    while remaining:
        h = remaining.pop(0)
        poly = _bridge(poly, h, remaining)

    pts = np.asarray(poly, dtype=float)
    idx = list(range(len(pts)))
    tris = []
    guard = 0
    i = 0
    while len(idx) > 3:
        guard += 1
        if guard > 50 * len(pts) + 1000:
            raise RuntimeError('ear clipping failed')
        n = len(idx)
        ia, ib, ic = idx[(i - 1) % n], idx[i % n], idx[(i + 1) % n]
        a, b, c = pts[ia], pts[ib], pts[ic]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if abs(cross) < 1e-12 and np.allclose(a, c):
            # zero-area spike from a bridge; drop the tip
            idx.pop(i % n)
            continue
        is_ear = cross > 1e-12
        if is_ear:
            others = pts[idx]
            v0, v1, v2 = c - a, b - a, others - a
            d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
            d20, d21 = v2 @ v0, v2 @ v1
            den = d00 * d11 - d01 * d01
            u = (d11 * d20 - d01 * d21) / den
            v = (d00 * d21 - d01 * d20) / den
            inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
            coincident = np.isclose(others, a).all(1) | np.isclose(others, b).all(1) | \
                np.isclose(others, c).all(1)
            is_ear = not (inside & ~coincident).any()
        if is_ear:
            tris.append((ia, ib, ic))
            idx.pop(i % n)
            guard = 0
        else:
            i += 1
    tris.append(tuple(idx))
    return pts, tris


# ---------------------------------------------------------------------------
# 3D meshes (list of triangles, each a (3, 3) array)
# ---------------------------------------------------------------------------
def extrude(outer, holes=(), z0=0.0, z1=1.0):
    pts, tris = triangulate(outer, holes)
    out = []
    for (a, b, c) in tris:
        pa, pb, pc = pts[a], pts[b], pts[c]
        out.append(np.array([[*pa, z1], [*pb, z1], [*pc, z1]]))   # top (CCW from above)
        out.append(np.array([[*pa, z0], [*pc, z0], [*pb, z0]]))   # bottom
    outer = outer if _area(outer) > 0 else list(reversed(outer))
    rings = [outer] + [h if _area(h) < 0 else list(reversed(h)) for h in holes]
    for ring in rings:
        for k in range(len(ring)):
            (x1, y1), (x2, y2) = ring[k], ring[(k + 1) % len(ring)]
            out.append(np.array([[x1, y1, z0], [x2, y2, z0], [x2, y2, z1]]))
            out.append(np.array([[x1, y1, z0], [x2, y2, z1], [x1, y1, z1]]))
    return out


def transform(mesh, matrix=np.eye(3), offset=(0, 0, 0)):
    m = np.asarray(matrix, dtype=float)
    flip = np.linalg.det(m) < 0
    out = []
    for t in mesh:
        t2 = t @ m.T + np.asarray(offset, dtype=float)
        out.append(t2[[0, 2, 1]] if flip else t2)
    return out


# Maps a profile drawn in (x, z) and extruded along +w into world (x, y=w, z)
XZ_PROFILE = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]])


def write_stl(path, mesh, name):
    with open(path, 'wb') as f:
        header = f'{name} - rover parametric part (mm)'.encode()[:80]
        f.write(header.ljust(80, b' '))
        f.write(struct.pack('<I', len(mesh)))
        for t in mesh:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(n)
            n = n / ln if ln > 0 else n
            f.write(struct.pack('<12fH', *n, *t[0], *t[1], *t[2], 0))


def check_mesh(mesh, name):
    """Every directed edge must be matched by its reverse (closed, oriented)."""
    key = lambda p: tuple(np.round(p, 5))
    edges = {}
    volume = 0.0
    for t in mesh:
        volume += np.dot(t[0], np.cross(t[1], t[2])) / 6.0
        for i in range(3):
            e = (key(t[i]), key(t[(i + 1) % 3]))
            edges[e] = edges.get(e, 0) + 1
    bad = sum(1 for (a, b), n in edges.items() if edges.get((b, a), 0) != n)
    status = 'OK' if bad == 0 and volume > 0 else 'FAIL'
    print(f'  [{status}] {name:15s} {len(mesh):6d} triangles, volume {volume / 1000:8.1f} cm3'
          + (f', {bad} unmatched edges' if bad else ''))
    return status == 'OK'


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------
class Rover:
    def __init__(self, dims):
        mm = lambda v: v * 1000.0
        c, w, mo, ca, li, cam = (dims['chassis'], dims['wheel'], dims['motor'],
                                 dims['caster'], dims['lidar'], dims['camera'])
        self.L, self.W = mm(c['length']), mm(c['width'])
        self.t = mm(c['plate_thickness'])
        self.r_corner = mm(c['corner_radius'])
        self.inset = mm(c['standoff_inset'])
        self.deck = mm(c['deck_spacing'])
        self.plate_z = mm(c['lower_plate_z'])
        self.wheel_r, self.wheel_w = mm(w['radius']), mm(w['width'])
        self.tire_t = mm(w['tire_thickness'])
        self.shaft_d, self.shaft_flat = mm(w['shaft_diameter']), mm(w['shaft_flat'])
        self.motor_d = mm(mo['diameter'])
        self.motor_holes = mm(mo['mount_hole_spacing'])
        self.caster_x = mm(ca['x_offset'])
        self.caster_h = mm(ca['body_height'])
        self.lidar_x, self.lidar_size = mm(li['x']), mm(li['size'])
        self.lidar_riser_h = mm(li['riser_height'])
        self.cam_x, self.cam_w = mm(cam['x']), mm(cam['width'])
        self.cam_riser_h = mm(cam['riser_height'])
        # derived
        self.axle_drop = self.plate_z - self.wheel_r          # plate bottom -> motor axis
        self.bracket_t = 4.0
        self.bracket_w = 32.0
        self.bracket_flange = 24.0

    # ---- hole patterns (plate coordinates, mm) ---------------------------
    def standoff_holes(self):
        x, y = self.L / 2 - self.inset, self.W / 2 - self.inset
        return [circle(sx * x, sy * y, M3 / 2) for sx in (1, -1) for sy in (1, -1)]

    def bracket_holes(self):
        y = self.W / 2 - self.bracket_flange / 2 - 2.0
        return [circle(sx * 10.0, sy * y, M3 / 2) for sx in (1, -1) for sy in (1, -1)]

    def caster_holes(self):
        return [circle(sx * self.caster_x, sy * 12.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]

    # ---- parts ------------------------------------------------------------
    def base_plate(self):
        holes = self.standoff_holes() + self.bracket_holes() + self.caster_holes()
        # Raspberry Pi 5 (58 x 49 mm hole pattern), rear half
        pi_cx = -50.0
        holes += [circle(pi_cx + sx * 29.0, sy * 24.5, M25 / 2) for sx in (1, -1) for sy in (1, -1)]
        # battery strap slots (front half)
        holes += [slot(27.5, sy * (self.W / 2 - 17.0), 26.0, 4.0) for sy in (1, -1)]
        # motor / encoder cable pass-through
        holes += [slot(0.0, sy * 22.0, 14.0, 9.0, angle_deg=90) for sy in (1, -1)]
        # camera riser: 2x M3 + 1/4" tripod screw access
        holes += [circle(self.cam_x, sy * 25.0, M3 / 2) for sy in (1, -1)]
        holes += [circle(self.cam_x, 0.0, QUARTER / 2)]
        outer = rounded_rect(0, 0, self.L, self.W, self.r_corner)
        return extrude(outer, holes, 0, self.t)

    def top_plate(self):
        holes = self.standoff_holes()
        lx = self.lidar_x
        holes += [circle(lx + sx * 26.0, sy * 26.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        holes += [circle(lx, 0.0, 10.0)]                                   # lidar cable
        # accessory grid (motor driver, buck converter, switch...)
        for x in (50.0, 70.0, -70.0, -55.0):
            if abs(x - lx) < 45:
                continue
            for y in (-40.0, -20.0, 0.0, 20.0, 40.0):
                holes.append(circle(x, y, M3 / 2))
        holes += [slot(85.0, 0.0, 30.0, 8.0, angle_deg=90)]                 # front cable slot
        outer = rounded_rect(0, 0, self.L, self.W, self.r_corner)
        return extrude(outer, holes, 0, self.t)

    def wheel_hub(self):
        """Printed outer face down. z = 0 is the OUTER face, z = width faces the chassis."""
        r_rim = self.wheel_r - self.tire_t
        web_t = 4.0
        hub_r = 7.0
        d_hole = d_shape(0, 0, self.shaft_d + 0.2, self.shaft_flat)
        spokes = [circle(18.5 * math.cos(a), 18.5 * math.sin(a), 5.5)
                  for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)]
        mesh = []
        mesh += extrude(circle(0, 0, r_rim - 0.01, 96), [circle(0, 0, r_rim - 3.0, 96)], 0, self.wheel_w)
        # web overlaps rim and hub slightly so slicers merge the shells
        mesh += extrude(circle(0, 0, r_rim - 2.5, 96), spokes + [circle(0, 0, hub_r - 0.5, 40)], 0, web_t)
        mesh += extrude(circle(0, 0, hub_r, 40), [d_hole], 0, self.wheel_w)
        return mesh

    def tire_tpu(self):
        r_rim = self.wheel_r - self.tire_t
        return extrude(circle(0, 0, self.wheel_r, 120), [circle(0, 0, r_rim - 0.2, 120)], 0, self.wheel_w)

    def motor_bracket(self):
        """L bracket. Print flange-down. Flange bolts under the lower plate, the wall holds
        the gearbox face (2x M3 + 7 mm boss). Print frame: y = 0 is the outer face."""
        w, t, fl = self.bracket_w, self.bracket_t, self.bracket_flange
        wall_h = self.axle_drop + self.motor_d / 2 + 4.0
        # flange (on the bed)
        flange_holes = [circle(sx * 10.0, fl / 2 + 2.0, M3 / 2) for sx in (1, -1)]
        mesh = extrude(rect(0, fl / 2, w, fl), flange_holes, 0, t)
        # wall (profile in x/z, thickness along +y)
        prof = rounded_rect(0, wall_h / 2, w, wall_h, 1.0, 2)
        prof = [(x, max(z, 0.0)) for x, z in rect(0, wall_h / 2, w, wall_h)]
        holes = [circle(0, self.axle_drop, 3.75), ] + \
                [circle(sx * self.motor_holes / 2, self.axle_drop, M3 / 2) for sx in (1, -1)]
        wall = extrude(prof, holes, 0, t)
        mesh += transform(wall, XZ_PROFILE)
        return mesh

    def caster_spacer(self):
        h = self.plate_z - self.caster_h
        holes = [circle(0, sy * 12.0, M3 / 2) for sy in (1, -1)]
        return extrude(rounded_rect(0, 0, 18.0, 36.0, 4.0), holes, 0, h)

    def lidar_riser(self):
        s = self.lidar_size + 8.0
        holes = [circle(sx * 26.0, sy * 26.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        holes += [circle(0, 0, 10.0)]
        # lighten
        holes += [slot(0, sy * 17.0, 26.0, 7.0) for sy in (1, -1)]
        return extrude(rounded_rect(0, 0, s, s, 6.0), holes, 0, self.lidar_riser_h)

    def camera_riser(self):
        holes = [circle(0, sy * 25.0, M3 / 2) for sy in (1, -1)] + [circle(0, 0, QUARTER / 2)]
        return extrude(rounded_rect(0, 0, 20.0, 64.0, 4.0), holes, 0, self.cam_riser_h)

    def standoff(self):
        return extrude(circle(0, 0, 3.5, 24), [circle(0, 0, 1.6, 16)], 0, self.deck)


PARTS = {
    # name: (quantity, material, note)
    'base_plate': (1, 'PETG/PLA', '100% flat, 4 perimeters, 30% infill'),
    'top_plate': (1, 'PETG/PLA', '100% flat, 4 perimeters, 30% infill'),
    'wheel_hub': (2, 'PETG', 'outer face down, 5 perimeters for the D-hole'),
    'tire_tpu': (2, 'TPU 95A', 'stretch over the hub rim'),
    'motor_bracket': (2, 'PETG', 'flange down, 40% infill'),
    'caster_spacer': (2, 'PETG/PLA', ''),
    'lidar_riser': (1, 'PETG/PLA', ''),
    'camera_riser': (1, 'PETG/PLA', ''),
    'standoff': (4, 'PETG/PLA', 'or use M3x60 brass standoffs'),
}


# ---------------------------------------------------------------------------
# Rover4: 4WD + front (Ackermann) steering
#
# Parts are modelled in the robot's own frames (millimetres):
#   base_link  = rear-axle centre, z relative to the axle   (plates, beam, mounts)
#   steering   = kingpin axis of the left front knuckle      (knuckles)
#   wheel      = wheel centre, axle along y                  (hubs, tires)
# so the same geometry is used as an exact visual mesh in the simulator; for printing
# each part is rotated onto its flat face and moved onto the bed.
# ---------------------------------------------------------------------------
RX_PI = np.diag([1.0, -1.0, -1.0])                       # 180 deg about x (flip upside down)
MIRROR_Y = np.diag([1.0, -1.0, 1.0])                     # left <-> right
WHEEL_Z_TO_Y = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)  # axis z -> -y


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def arc(cx, cy, r, a0_deg, a1_deg, n=8):
    return [(cx + r * math.cos(math.radians(a0_deg + (a1_deg - a0_deg) * i / n)),
             cy + r * math.sin(math.radians(a0_deg + (a1_deg - a0_deg) * i / n))) for i in range(n + 1)]


def for_print(mesh, rotation=np.eye(3)):
    """Rotate a part onto its flat face and put it on the bed, centred at x = y = 0."""
    m = transform(mesh, rotation)
    pts = np.concatenate(m)
    lo, hi = pts.min(0), pts.max(0)
    return transform(m, np.eye(3), (-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]))


class Rover4:
    def __init__(self, dims):
        mm = lambda v: v * 1000.0
        c, w, a, st, mo, li, cam = (dims['chassis'], dims['wheel'], dims['axles'], dims['steering'],
                                    dims['motor'], dims['lidar'], dims['camera'])
        self.rear_x, self.front_x = mm(c['rear_x']), mm(c['front_x'])
        self.W, self.front_W = mm(c['width']), mm(c['front_width'])
        self.narrow_x, self.split_x = mm(c['narrow_from_x']), mm(c['split_x'])
        self.t, self.r_corner = mm(c['plate_thickness']), mm(c['corner_radius'])
        self.deck = mm(c['deck_spacing'])
        self.top_L, self.top_W, self.top_x = mm(c['top_length']), mm(c['top_width']), mm(c['top_x'])
        self.wheel_r, self.wheel_w = mm(w['radius']), mm(w['width'])
        self.tire_t = mm(w['tire_thickness'])
        self.shaft_d, self.shaft_flat = mm(w['shaft_diameter']), mm(w['shaft_flat'])
        self.wheelbase, self.track = mm(a['wheelbase']), mm(a['track'])
        self.kingpin_y, self.arm_len = mm(st['kingpin_y']), mm(st['arm_length'])
        self.servo_x, self.horn = mm(st['servo_x']), mm(st['horn_length'])
        self.motor_w, self.motor_h, self.motor_len = mm(mo['width']), mm(mo['height']), mm(mo['length'])
        self.lidar_x, self.lidar_size = mm(li['x']), mm(li['size'])
        self.lidar_riser_h, self.lidar_holes = mm(li['riser_height']), mm(li['mount_hole_spacing'])
        self.cam_x, self.cam_riser_h = mm(cam['x']), mm(cam['riser_height'])
        # heights relative to the axle
        self.z_low_bot = mm(c['lower_plate_z']) - self.wheel_r
        self.z_low_top = self.z_low_bot + self.t
        self.z_up_bot = self.z_low_top + self.deck
        self.z_up_top = self.z_up_bot + self.t
        self.beam_t = 6.0
        self.z_beam_bot = self.z_low_bot - self.beam_t
        self.tab_t = 5.0
        self.wheel_offset = self.track / 2 - self.kingpin_y       # kingpin -> wheel centre (y)
        # Ackermann arm: points from the kingpin towards the rear-axle centre
        ang = math.atan2(-self.kingpin_y, -self.wheelbase)
        self.arm_end = (self.arm_len * math.cos(ang), self.arm_len * math.sin(ang))
        # upper deck supports: rear corners, beside the battery, and either side of the servo
        self.standoffs = [(x, sy * y) for x, y in ((12.0, 63.0), (112.0, 66.0), (185.0, 30.0))
                          for sy in (1, -1)]

    # ---- lower deck -------------------------------------------------------
    def _deck_holes(self):
        h = [circle(x, y, M3 / 2) for x, y in self.standoffs]
        h += [circle(x, y, M3 / 2) for x in (80.0, 120.0) for y in (-45.0, 0.0, 45.0)]      # splice
        h += [circle(sx * 19.0, sy * 68.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]      # rear clamps
        h += [circle(40.0 + sx * 29.0, sy * 24.5, M25 / 2) for sx in (1, -1) for sy in (1, -1)]  # Pi 5
        h += [slot(25.0, sy * 50.0, 16.0, 8.0) for sy in (1, -1)]                            # cables
        h += [slot(114.0, sy * 58.0, 24.0, 4.0) for sy in (1, -1)]                           # battery strap
        h += [rect(self.servo_x - 10.0, 0.0, 42.0, 21.0)]                                    # servo body
        h += [circle(self.servo_x - 10.0 + sx * 27.0, sy * 12.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        h += [circle(self.wheelbase, sy * 30.0, M3 / 2) for sy in (1, -1)]                   # front beam
        h += [circle(self.cam_x, sy * 25.0, M3 / 2) for sy in (1, -1)] + [circle(self.cam_x, 0.0, QUARTER / 2)]
        return h

    def _deck_outline(self, x0, x1):
        """Outline of the lower deck between x0 and x1 (the seam edges are square)."""
        W2, F2, r = self.W / 2, self.front_W / 2, self.r_corner
        ramp = 30.0                                            # 45 deg transition to the nose
        pts = []
        # right side, rear -> front
        if x0 <= self.rear_x + 1e-6:
            pts += arc(self.rear_x + r, -W2 + r, r, 180, 270)
        else:
            pts.append((x0, -W2))
        if x1 >= self.front_x - 1e-6:
            pts += [(self.narrow_x - ramp, -W2), (self.narrow_x, -F2)]
            pts += arc(self.front_x - r, -F2 + r, r, 270, 360)
            pts += arc(self.front_x - r, F2 - r, r, 0, 90)
            pts += [(self.narrow_x, F2), (self.narrow_x - ramp, W2)]
        else:
            pts += [(x1, -W2), (x1, W2)]
        if x0 <= self.rear_x + 1e-6:
            pts += arc(self.rear_x + r, W2 - r, r, 90, 180)
        else:
            pts.append((x0, W2))
        if x0 > self.rear_x + 1e-6 and x1 < self.front_x:
            pass
        return pts

    def _deck_half(self, x0, x1):
        holes = [h for h in self._deck_holes() if all(x0 + 2 < p[0] < x1 - 2 for p in h)]
        return extrude(self._deck_outline(x0, x1), holes, self.z_low_bot, self.z_low_top)

    def base_rear(self):
        return self._deck_half(self.rear_x, self.split_x)

    def base_front(self):
        return self._deck_half(self.split_x, self.front_x)

    def splice_plate(self):
        holes = [circle(x, y, M3 / 2) for x in (80.0, 120.0) for y in (-45.0, 0.0, 45.0)]
        return extrude(rounded_rect(self.split_x, 0, 60.0, 120.0, 6.0), holes,
                       self.z_low_bot - 5.0, self.z_low_bot)

    # ---- upper deck -------------------------------------------------------
    def top_plate(self):
        lx = self.lidar_x
        holes = [circle(x, y, M3 / 2) for x, y in self.standoffs]
        holes += [circle(lx + sx * 36.0, sy * 36.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        holes += [circle(lx, 0.0, 12.0)]
        for x in (20.0, 40.0, 160.0, 180.0):
            for y in (-40.0, -20.0, 0.0, 20.0, 40.0):
                if all(math.hypot(x - sx, y - sy) > 8 for sx, sy in self.standoffs):
                    holes.append(circle(x, y, M3 / 2))
        outer = rounded_rect(self.top_x, 0, self.top_L, self.top_W, self.r_corner)
        return extrude(outer, holes, self.z_up_bot, self.z_up_top)

    # ---- drive & steering -------------------------------------------------
    def rear_motor_clamp(self):
        """Left rear N20 clamp (the right one is the same part turned around)."""
        y0, y1 = 58.0, 82.0
        prof = rect(0.0, (self.z_low_bot - 8.0) / 2 - 4.0 + 4.0, 26.0, self.z_low_bot + 8.0)
        prof = [(x, min(z, self.z_low_bot)) for x, z in prof]
        pocket = rect(0.0, 0.0, self.motor_w + 0.3, self.motor_h + 0.3)
        body = transform(extrude(prof, [pocket], y0, y1), XZ_PROFILE)
        ears = extrude(rect(0.0, 70.0, 48.0, 24.0), [circle(sx * 19.0, 68.0, M3 / 2) for sx in (1, -1)],
                       self.z_low_bot - 5.0, self.z_low_bot)
        return body + ears

    def front_beam(self):
        k = self.kingpin_y
        outer = convex_hull(circle(self.wheelbase, k, 8.0, 24) + circle(self.wheelbase, -k, 8.0, 24))
        holes = [circle(self.wheelbase, sy * k, 2.1) for sy in (1, -1)]                     # M4 kingpins
        holes += [circle(self.wheelbase, sy * 30.0, M3 / 2) for sy in (1, -1)]
        return extrude(outer, holes, self.z_beam_bot, self.z_low_bot)

    def knuckle_left(self):
        """Left front knuckle in its steering frame (origin on the kingpin axis)."""
        ax, ay = self.arm_end
        z_top = self.z_beam_bot
        z_tab = z_top - self.tab_t
        tab = convex_hull(circle(0, 0, 8.0, 24) + circle(ax, ay, 5.0, 16) + rect(0.0, 5.0, 24.0, 10.0))
        mesh = extrude(tab, [circle(0, 0, 2.1), circle(ax, ay, M3 / 2)], z_tab, z_top)
        wall = [(x, z) for x, z in rect(0.0, (z_top - 8.0) / 2 - 4.0 + 4.0, 24.0, z_top + 8.0)]
        wall = [(x, min(max(z, -8.0), z_top)) for x, z in wall]
        pocket = rect(0.0, 0.0, self.motor_w + 0.3, self.motor_h + 0.3)
        mesh += transform(extrude(wall, [pocket], 0.0, 10.0), XZ_PROFILE)
        return mesh

    def knuckle_right(self):
        return transform(self.knuckle_left(), MIRROR_Y)

    def tie_rod(self):
        ax, ay = self.arm_end
        pin = (self.servo_x - self.horn, 0.0)
        end = (self.wheelbase + ax, self.kingpin_y + ay)
        length = math.hypot(end[0] - pin[0], end[1] - pin[1])
        return extrude(slot(0, 0, length + 8.0, 8.0),
                       [circle(-length / 2, 0, M3 / 2), circle(length / 2, 0, M3 / 2)], 0, 4.0)

    def servo_mount(self):
        cx = self.servo_x - 10.0
        holes = [rect(cx, 0, 41.0, 20.5)]
        holes += [circle(cx + sx * 24.75, sy * 5.0, 2.1) for sx in (1, -1) for sy in (1, -1)]   # servo ears
        holes += [circle(cx + sx * 27.0, sy * 12.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        return extrude(rounded_rect(cx, 0, 62.0, 32.0, 4.0), holes, self.z_low_top, self.z_low_top + 12.0)

    # ---- wheels -----------------------------------------------------------
    def _hub_z(self):
        r_rim = self.wheel_r - self.tire_t
        d_hole = d_shape(0, 0, self.shaft_d + 0.2, self.shaft_flat)
        spokes = [circle(22.0 * math.cos(a), 22.0 * math.sin(a), 7.0)
                  for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)]
        mesh = extrude(circle(0, 0, r_rim - 0.01, 96), [circle(0, 0, r_rim - 3.0, 96)], 0, self.wheel_w)
        mesh += extrude(circle(0, 0, r_rim - 2.5, 96), spokes + [circle(0, 0, 6.5, 40)], 0, 4.0)
        mesh += extrude(circle(0, 0, 7.0, 40), [d_hole], 0, self.wheel_w)
        return mesh

    def _tire_z(self):
        r_rim = self.wheel_r - self.tire_t
        return extrude(circle(0, 0, self.wheel_r, 120), [circle(0, 0, r_rim - 0.2, 120)], 0, self.wheel_w)

    def wheel_hub(self):     # wheel frame: axle along y, outer face at +y
        return transform(self._hub_z(), WHEEL_Z_TO_Y, (0, self.wheel_w / 2, 0))

    def tire_tpu(self):
        return transform(self._tire_z(), WHEEL_Z_TO_Y, (0, self.wheel_w / 2, 0))

    # ---- sensor mounts ----------------------------------------------------
    def lidar_riser(self):
        s = self.lidar_size + 8.0
        lx, hs = self.lidar_x, self.lidar_holes / 2
        holes = [circle(lx + sx * 36.0, sy * 36.0, M3 / 2) for sx in (1, -1) for sy in (1, -1)]
        holes += [circle(lx + sx * hs, sy * hs, M25 / 2) for sx in (1, -1) for sy in (1, -1)]
        holes += [circle(lx, 0, 12.0)]
        return extrude(rounded_rect(lx, 0, s, s, 8.0), holes, self.z_up_top, self.z_up_top + self.lidar_riser_h)

    def camera_riser(self):
        holes = [circle(self.cam_x, sy * 25.0, M3 / 2) for sy in (1, -1)] + [circle(self.cam_x, 0, QUARTER / 2)]
        return extrude(rounded_rect(self.cam_x, 0, 20.0, 64.0, 4.0), holes,
                       self.z_low_top, self.z_low_top + self.cam_riser_h)

    def standoff(self):
        return extrude(circle(0, 0, 3.5, 24), [circle(0, 0, 1.6, 16)], 0, self.deck)


# name: (quantity, material, note, print rotation, used as sim visual)
PARTS_ROVER4 = {
    'base_rear': (1, 'PETG', 'lower deck, rear half', np.eye(3), True),
    'base_front': (1, 'PETG', 'lower deck, front half (steering nose)', np.eye(3), True),
    'splice_plate': (1, 'PETG', 'joins the two deck halves from below', np.eye(3), True),
    'top_plate': (1, 'PETG/PLA', 'upper deck, lidar + electronics', np.eye(3), True),
    'front_beam': (1, 'PETG', 'front axle beam, M4 kingpins; 50 % infill', np.eye(3), True),
    'knuckle_left': (1, 'PETG', 'steering knuckle + N20 clamp; 5 perimeters', RX_PI, True),
    'knuckle_right': (1, 'PETG', 'mirror of the left knuckle', RX_PI, True),
    'rear_motor_clamp': (2, 'PETG', 'N20 clamp under the rear deck', RX_PI, True),
    'servo_mount': (1, 'PETG', 'MG996R frame, shaft points down through the deck', np.eye(3), True),
    'tie_rod': (2, 'PETG', 'servo horn -> knuckle arm, M3 shoulder screws', np.eye(3), False),
    'wheel_hub': (4, 'PETG', 'outer face down; 3 mm D-shaft press fit', None, True),
    'tire_tpu': (4, 'TPU 95A', 'stretch over the hub rim', None, True),
    'lidar_riser': (1, 'PETG/PLA', 'RPLIDAR S2M1: verify its hole pattern', np.eye(3), True),
    'camera_riser': (1, 'PETG/PLA', 'RealSense D435i, 1/4"-20 from below', np.eye(3), True),
    'standoff': (6, 'PETG/PLA', 'or M3 x 60 mm brass standoffs', np.eye(3), False),
}


def build_rover4(check):
    dims = yaml.safe_load(DIMS_FILE_ROVER4.read_text())
    r4 = Rover4(dims)
    out = STL_DIR / 'rover4'
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    print(f'Generating rover4 parts from {DIMS_FILE_ROVER4}')
    for name, (qty, material, note, rot, visual) in PARTS_ROVER4.items():
        mesh = getattr(r4, name)()
        if name == 'wheel_hub':
            printable = for_print(r4._hub_z())
        elif name == 'tire_tpu':
            printable = for_print(r4._tire_z())
        else:
            printable = for_print(mesh, rot)
        write_stl(out / f'{name}.stl', printable, f'rover4 {name}')
        if visual:
            write_stl(MESH_DIR / f'rover4_{name}.stl', mesh, f'rover4 {name} (assembly frame)')
        if check:
            ok &= check_mesh(printable, name)
        else:
            print(f'  {name:17s} x{qty}  {material:9s} {note}')
    print(f'STL files written to {out}')
    return ok


def build_rover(check):
    dims = yaml.safe_load(DIMS_FILE.read_text())
    rover = Rover(dims)
    STL_DIR.mkdir(exist_ok=True)
    ok = True
    print(f'Generating rover parts from {DIMS_FILE}')
    for name, (qty, material, note) in PARTS.items():
        mesh = getattr(rover, name)()
        path = STL_DIR / f'{name}.stl'
        write_stl(path, mesh, name)
        if name in VISUAL_PARTS:
            shutil.copy(path, MESH_DIR / path.name)
        if check:
            ok &= check_mesh(mesh, name)
        else:
            print(f'  {name:15s} x{qty}  {material:9s} {note}')
    print(f'STL files written to {STL_DIR}')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='verify meshes are closed')
    ap.add_argument('--robot', choices=['rover', 'rover4', 'all'], default='all',
                    help='which robot to generate (default: all)')
    args = ap.parse_args()
    MESH_DIR.mkdir(exist_ok=True)
    ok = True
    if args.robot in ('rover', 'all'):
        ok &= build_rover(args.check)
    if args.robot in ('rover4', 'all'):
        ok &= build_rover4(args.check)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
