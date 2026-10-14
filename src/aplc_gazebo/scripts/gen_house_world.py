#!/usr/bin/env python3
"""Generate a Gazebo Harmonic world: a two-storey house raised on a foundation,
with a furnished interior, an open front door at a landing, a straight wooden
ramp down along the front wall, a concrete apron, an asphalt driveway, a street,
and a parked car.

Layout (default, meters, z up):
  - Front wall on the plane x = 0; the house extends toward -x (depth 8 m), y from -5 to 5.
  - Ground floor at z = --rise (the door threshold and landing height).
  - Ground floor (open plan): living room (sofa, coffee table, TV), kitchen island
    with DROP-OFF 1 on its countertop, wall counter and fridge, dining table, bookshelf.
  - Straight staircase along the back wall up to the first floor.
  - First floor: stairwell with guard rails, hallway, partition wall with a doorway,
    bedroom (bed, nightstand, dresser, desk) with DROP-OFF 2 on the floor inside the doorway.
  - Landing and ramp outside the door, railing on the outer side.
  - Car: a true-scale Toyota RAV4 (5th gen) built from primitives, rear toward the house.
    Its origin is the rear bumper center at ground level, +x pointing to the car's front.

Examples:
  python3 gen_house_world.py -o house_ramp.sdf
  python3 gen_house_world.py --rise 0.6 --slope 15 -o house_ramp_tall.sdf
  python3 gen_house_world.py --house solid -o house_solid.sdf      # no interior
  python3 gen_house_world.py --car-x 8 --car-y -3.0 --car-yaw 10 -o house_car_moved.sdf
  # Installed worlds: no car (world.launch.py spawns models/rav4.sdf at car_x/car_y/car_yaw)
  python3 gen_house_world.py --car none --car-file rav4.sdf -o house_1.sdf
"""
import argparse
import math
import os

# A bare output filename is written into the package's worlds/ folder (next to scripts/).
WORLDS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds"))
# A bare --car-file name is written into the package's models/ folder.
MODELS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models"))

# ----------------------------------------------------------------------------- colors
GRASS = (0.30, 0.52, 0.24)
CONCRETE = (0.70, 0.70, 0.67)
ASPHALT = (0.20, 0.20, 0.22)
STREET = (0.13, 0.13, 0.14)
WALL = (0.86, 0.85, 0.81)
INNER_WALL = (0.93, 0.92, 0.88)
BRICK = (0.55, 0.26, 0.20)
ROOF = (0.24, 0.24, 0.26)
DOOR = (0.36, 0.20, 0.12)
GLASS = (0.22, 0.32, 0.42)
WOOD = (0.56, 0.33, 0.17)
RAIL = (0.50, 0.29, 0.14)
FLOOR = (0.66, 0.50, 0.33)
STAIR = (0.58, 0.42, 0.26)
CABINET = (0.90, 0.90, 0.87)
COUNTERTOP = (0.25, 0.25, 0.27)
STEEL = (0.74, 0.75, 0.77)
SOFA = (0.35, 0.40, 0.50)
DARK_WOOD = (0.40, 0.26, 0.15)
RUG = (0.55, 0.30, 0.25)
BLACK = (0.05, 0.05, 0.05)
BEDDING = (0.85, 0.86, 0.90)
DROPOFF = (1.00, 0.45, 0.00)
CAR_PAINT = (0.16, 0.27, 0.45)
TIRE = (0.08, 0.08, 0.08)

FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
CARS = {"suv": "SUV", "pickup": "Pickup", "hatchback": "Hatchback", "prius": "Prius Hybrid"}

# Toyota RAV4, 5th generation (2019+), published exterior dimensions (m).
# Overhangs, clearance and beltline are approximate; the cargo floor height is
# what the trunk lift/conveyor must meet, so measure the real car before relying on it.
RAV4 = dict(
    length=4.600, width=1.855, height=1.685, wheelbase=2.690, track=1.600,
    rear_overhang=0.985, ground_clearance=0.21, tire_radius=0.362, tire_width=0.225,
    beltline=1.05, cargo_floor=0.75,
)


def _geom(size):
    return "<geometry><box><size>{:.4f} {:.4f} {:.4f}</size></box></geometry>".format(*size)


def _mat(color):
    c = "{:.3f} {:.3f} {:.3f} 1".format(*color)
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>"


def _pose(p):
    return "<pose>{:.4f} {:.4f} {:.4f} {:.5f} {:.5f} {:.5f}</pose>".format(*p)


def box(name, size, pose, color, collision=True):
    """A static model with one box. pose = (x, y, z, roll, pitch, yaw), SDF convention."""
    col = f'\n        <collision name="collision">{_geom(size)}</collision>' if collision else ""
    return f"""
    <model name="{name}">
      <static>true</static>
      {_pose(pose)}
      <link name="link">{col}
        <visual name="visual">{_geom(size)}{_mat(color)}</visual>
      </link>
    </model>"""


def compound(name, parts):
    """A static model made of several boxes, all given in world coordinates.
    part = (size, pose, color) or (size, pose, color, collision_flag)."""
    out = []
    for i, part in enumerate(parts):
        size, pose, color = part[:3]
        collide = part[3] if len(part) > 3 else True
        if collide:
            out.append(f'<collision name="c{i}">{_pose(pose)}{_geom(size)}</collision>')
        out.append(f'<visual name="v{i}">{_pose(pose)}{_geom(size)}{_mat(color)}</visual>')
    body = "\n        ".join(out)
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
        {body}
      </link>
    </model>"""


def aabb(x0, x1, y0, y1, z0, z1, color, collision=True):
    """A box part from its extents (world frame, axis aligned)."""
    return ((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 0, 0, 0), color, collision)


def table_parts(x0, x1, y0, y1, zf, h, color, top_t=0.04, leg=0.05):
    parts = [aabb(x0, x1, y0, y1, zf + h - top_t, zf + h, color)]
    for lx in (x0 + leg / 2 + 0.03, x1 - leg / 2 - 0.03):
        for ly in (y0 + leg / 2 + 0.03, y1 - leg / 2 - 0.03):
            parts.append(aabb(lx - leg / 2, lx + leg / 2, ly - leg / 2, ly + leg / 2, zf, zf + h - top_t, color))
    return parts


def chair_parts(xc, yc, zf, back_side, color, s=0.42):
    """back_side: '+x', '-x', '+y' or '-y' (where the backrest is)."""
    parts = table_parts(xc - s / 2, xc + s / 2, yc - s / 2, yc + s / 2, zf, 0.46, color, top_t=0.04, leg=0.04)
    bt, bh = 0.04, 0.45
    z0, z1 = zf + 0.46, zf + 0.46 + bh
    if back_side == "+y":
        parts.append(aabb(xc - s / 2, xc + s / 2, yc + s / 2 - bt, yc + s / 2, z0, z1, color))
    elif back_side == "-y":
        parts.append(aabb(xc - s / 2, xc + s / 2, yc - s / 2, yc - s / 2 + bt, z0, z1, color))
    elif back_side == "+x":
        parts.append(aabb(xc + s / 2 - bt, xc + s / 2, yc - s / 2, yc + s / 2, z0, z1, color))
    else:
        parts.append(aabb(xc - s / 2, xc - s / 2 + bt, yc - s / 2, yc + s / 2, z0, z1, color))
    return parts


def wall_along_y(xc, t, y0, y1, z0, z1, color, openings=()):
    """Wall parts for a wall running along y, with rectangular openings (yc, width, z_bottom, z_top)."""
    parts, cur = [], y0
    for yc, w, zb, zt in sorted(openings):
        a, b = yc - w / 2, yc + w / 2
        if a > cur:
            parts.append(aabb(xc - t / 2, xc + t / 2, cur, a, z0, z1, color))
        if zb > z0:
            parts.append(aabb(xc - t / 2, xc + t / 2, a, b, z0, zb, color))
        if zt < z1:
            parts.append(aabb(xc - t / 2, xc + t / 2, a, b, zt, z1, color))
        cur = b
    if y1 > cur:
        parts.append(aabb(xc - t / 2, xc + t / 2, cur, y1, z0, z1, color))
    return parts


def rail_run(x0, y0, x1, y1, zf, color, heights=(0.10, 0.50, 0.95), post=0.06, spacing=1.2):
    """Axis-aligned guard rail from (x0, y0) to (x1, y1) on a floor at zf."""
    parts = []
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(1, math.ceil(length / spacing))
    for i in range(n + 1):
        px, py = x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n
        parts.append(aabb(px - post / 2, px + post / 2, py - post / 2, py + post / 2, zf, zf + max(heights) + 0.05, color))
    for hz in heights:
        if abs(x1 - x0) > abs(y1 - y0):
            parts.append(aabb(min(x0, x1), max(x0, x1), y0 - 0.02, y0 + 0.02, zf + hz - 0.03, zf + hz + 0.03, color))
        else:
            parts.append(aabb(x0 - 0.02, x0 + 0.02, min(y0, y1), max(y0, y1), zf + hz - 0.03, zf + hz + 0.03, color))
    return parts


def include(uri, name, pose):
    return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <static>true</static>
      {_pose(pose)}
    </include>"""


def rav4_model(name="car", pose=(0, 0, 0, 0, 0, 0)):
    """A static RAV4 at true scale. Origin: rear bumper center on the ground, +x toward the front."""
    c = RAV4
    L, W, H = c["length"], c["width"], c["height"]
    ra, fa = c["rear_overhang"], c["rear_overhang"] + c["wheelbase"]   # rear / front axle x
    r, tw, gc, bl = c["tire_radius"], c["tire_width"], c["ground_clearance"], c["beltline"]
    ah = r + 0.05                          # wheel arch half-length
    arch_top = 2 * r + 0.03
    cabin_x1 = L - 1.25                    # windshield base; the hood is in front of it

    def part(x0, x1, z0, z1, wdt, color):
        return ((x1 - x0, wdt, z1 - z0), ((x0 + x1) / 2, 0, (z0 + z1) / 2, 0, 0, 0), color)

    boxes = [
        # lower body between the wheel arches
        part(0, ra - ah, gc, arch_top, W, CAR_PAINT),
        part(ra + ah, fa - ah, gc, arch_top, W, CAR_PAINT),
        part(fa + ah, L, gc, arch_top, W, CAR_PAINT),
        part(0, L, arch_top, bl, W, CAR_PAINT),                       # body above the arches, hood level
        part(0.12, cabin_x1, bl, H - 0.06, W - 0.20, GLASS),          # glasshouse
        part(0.12, cabin_x1, H - 0.06, H, W - 0.20, CAR_PAINT),       # roof
    ]
    out = []
    for i, (size, p, color) in enumerate(boxes):
        out.append(f'<collision name="c{i}">{_pose(p)}{_geom(size)}</collision>')
        out.append(f'<visual name="v{i}">{_pose(p)}{_geom(size)}{_mat(color)}</visual>')
    cyl = f"<geometry><cylinder><radius>{r:.4f}</radius><length>{tw:.4f}</length></cylinder></geometry>"
    for i, (wx, wy) in enumerate([(ra, 1), (ra, -1), (fa, 1), (fa, -1)]):
        p = _pose((wx, wy * (c["track"] / 2), r, math.pi / 2, 0, 0))
        out.append(f'<collision name="wheel_c{i}">{p}{cyl}</collision>')
        out.append(f'<visual name="wheel_v{i}">{p}{cyl}{_mat(TIRE)}</visual>')
    body = "\n        ".join(out)
    return f"""
    <model name="{name}">
      <static>true</static>
      {_pose(pose)}
      <link name="link">
        {body}
      </link>
    </model>"""


def point_light(name, x, y, z):
    return f"""
    <light type="point" name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <diffuse>0.55 0.53 0.50 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>14</range><constant>0.6</constant><linear>0.04</linear><quadratic>0.005</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", default="house_ramp.sdf")
    # ramp and landing
    ap.add_argument("--rise", type=float, default=0.45, help="door threshold / landing height (m)")
    ap.add_argument("--slope", type=float, default=12.0, help="horizontal run per unit rise (12 = 1:12)")
    ap.add_argument("--ramp-width", type=float, default=1.2)
    ap.add_argument("--landing-length", type=float, default=1.5, help="landing length along the wall (m)")
    ap.add_argument("--landing-depth", type=float, default=1.52,
                    help="landing depth out from the wall (m); ADA asks 1.52 where the route turns")
    ap.add_argument("--ramp-side", choices=["right", "left"], default="right",
                    help="direction the ramp descends from the door, seen facing the house")
    ap.add_argument("--no-railing", action="store_true")
    # house
    ap.add_argument("--house", choices=["interior", "solid", "fuel"], default="interior")
    ap.add_argument("--door-width", type=float, default=0.91, help="clear doorway width (m)")
    ap.add_argument("--storey-height", type=float, default=2.9, help="floor-to-floor height (m)")
    ap.add_argument("--fuel-model", default="House 1")
    ap.add_argument("--fuel-x", type=float, default=-5.0)
    ap.add_argument("--fuel-y", type=float, default=0.0)
    ap.add_argument("--fuel-yaw", type=float, default=0.0, help="degrees")
    # site
    ap.add_argument("--apron-width", type=float, default=2.0)
    ap.add_argument("--driveway-length", type=float, default=12.0)
    ap.add_argument("--driveway-width", type=float, default=3.5)
    ap.add_argument("--car", choices=["rav4"] + list(CARS) + ["none"], default="rav4",
                    help="rav4 is built locally at true scale; the others come from Gazebo Fuel")
    ap.add_argument("--car-x", type=float, default=7.0,
                    help="car origin x (m); for rav4 the rear bumper center")
    ap.add_argument("--car-y", type=float, default=None, help="car origin y (m); default: driveway center")
    ap.add_argument("--car-yaw", type=float, default=0.0, help="degrees; 0 = front toward the street")
    ap.add_argument("--car-file", default=None,
                    help="also write the rav4 as a standalone model SDF (bare name: into models/)")
    a = ap.parse_args()
    if os.path.dirname(a.output) == "":
        os.makedirs(WORLDS_DIR, exist_ok=True)
        a.output = os.path.join(WORLDS_DIR, a.output)
    if a.car_file and os.path.dirname(a.car_file) == "":
        os.makedirs(MODELS_DIR, exist_ok=True)
        a.car_file = os.path.join(MODELS_DIR, a.car_file)

    # The interior layout is designed for this footprint and door position.
    W, D, door_y, t = 10.0, 8.0, 3.5, 0.2
    rise, rw, L, dw_door = a.rise, a.ramp_width, a.landing_length, a.door_width
    Ld = max(a.landing_depth, rw)   # the ramp joins the landing along the wall side
    run = rise * a.slope
    th = math.atan2(rise, run)
    slope_len = math.hypot(run, rise)
    sgn = 1.0 if a.ramp_side == "right" else -1.0

    def my(y):                      # mirror ramp geometry about the door for --ramp-side left
        return door_y + sgn * (y - door_y)

    y_top = door_y - L / 2
    y_bot = y_top - run

    def deck_z(y):
        return rise * (y - y_bot) / run

    H = a.storey_height
    z1 = rise                       # ground floor
    z2 = rise + H                   # first floor
    z_roof = z2 + H if a.house == "interior" else rise + 2.7
    xi0, xi1 = -D + t, -t           # interior x extents
    yi0, yi1 = -W / 2 + t, W / 2 - t

    m = []
    notes = {}

    # ----------------------------------------------------------------- ground and site
    m.append(f"""
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>{_mat(GRASS)}</visual>
      </link>
    </model>""")

    ys_ramp = sorted([my(y_bot), my(y_top + L)])
    ap_y0 = min(-W / 2, ys_ramp[0]) - 0.5
    ap_y1 = max(W / 2, ys_ramp[1]) + 0.5
    pave_t, pave_z = 0.004, 0.002   # visual-only paving, no lip; ground plane carries collision
    m.append(box("apron", (a.apron_width, ap_y1 - ap_y0, pave_t),
                 (a.apron_width / 2, (ap_y0 + ap_y1) / 2, pave_z, 0, 0, 0), CONCRETE, collision=False))
    drv_y = my(y_bot) - sgn * (a.driveway_width / 2 - 1.0)
    dx0, dx1 = a.apron_width, a.apron_width + a.driveway_length
    m.append(box("driveway", (a.driveway_length, a.driveway_width, pave_t),
                 ((dx0 + dx1) / 2, drv_y, pave_z, 0, 0, 0), ASPHALT, collision=False))
    m.append(box("street", (7.0, 40.0, pave_t), (dx1 + 3.5, 0, pave_z, 0, 0, 0), STREET, collision=False))

    # ----------------------------------------------------------------- house shell
    m.append(box("foundation", (D, W, rise), (-D / 2, 0, rise / 2, 0, 0, 0), BRICK))

    if a.house == "interior":
        door_open = (door_y, dw_door, z1, z1 + 2.1)
        walls = []
        walls += wall_along_y(-t / 2, t, -W / 2, W / 2, z1, z_roof, WALL, [door_open])          # front, open doorway
        walls += wall_along_y(-D + t / 2, t, -W / 2, W / 2, z1, z_roof, WALL)                   # back
        walls.append(aabb(-D, 0, -W / 2, -W / 2 + t, z1, z_roof, WALL))                          # side -y
        walls.append(aabb(-D, 0, W / 2 - t, W / 2, z1, z_roof, WALL))                            # side +y
        m.append(compound("house_walls", walls))

        # windows: thin visual panels on both faces of the front wall
        win = []
        for wy, zc in [(-3.0, z1 + 1.5), (-0.6, z1 + 1.5), (-3.0, z2 + 1.5), (0.5, z2 + 1.5), (3.5, z2 + 1.5)]:
            for xf in (0.006, -t - 0.006):
                win.append(((0.01, 1.4, 1.2), (xf, wy, zc, 0, 0, 0), GLASS, False))
        m.append(compound("windows", win))

        # front door, swung fully open inward against the hinge side
        m.append(box("front_door_open", (dw_door, 0.04, 2.05),
                     (-t - dw_door / 2, door_y + dw_door / 2 + 0.03, z1 + 1.025, 0, 0, 0), DOOR))

        # ground floor finish (visual only, no lip)
        m.append(box("ground_floor_finish", (xi1 - xi0, yi1 - yi0, 0.004),
                     ((xi0 + xi1) / 2, 0, z1 + 0.002, 0, 0, 0), FLOOR, collision=False))

        # staircase along the back wall, rising toward +y
        n_steps = max(2, round(H / 0.18))
        sr, srun, sw = H / n_steps, 0.28, 1.0
        s_len = n_steps * srun
        hx0, hx1 = xi0, xi0 + sw          # stair / stairwell x extents
        hy1 = 3.48                        # top of the stairs
        hy0 = hy1 - s_len                 # foot of the stairs
        steps = []
        for i in range(n_steps):
            h = (i + 1) * sr
            steps.append(aabb(hx0, hx1, hy0 + i * srun, hy0 + (i + 1) * srun, z1, z1 + h, STAIR))
        # handrail on the open side of the stairs
        ang = math.atan2(sr, srun)
        rail_len = math.hypot(s_len, H)
        zc = z1 + sr + (s_len / 2) * (sr / srun) + 0.9
        steps.append(((0.05, rail_len, 0.06), (hx1 - 0.03, hy0 + s_len / 2, zc, ang, 0, 0), DARK_WOOD))
        for py, pz in [(hy0 + 0.1, z1 + sr), (hy1 - 0.1, z2)]:
            steps.append(aabb(hx1 - 0.06, hx1, py - 0.03, py + 0.03, pz, pz + 0.95, DARK_WOOD))
        m.append(compound("staircase", steps))
        notes["stairs"] = f"{n_steps} steps, rise {sr*100:.1f} cm, run {srun*100:.0f} cm, foot at x {(hx0+hx1)/2:.2f} y {hy0:.2f}"

        # first floor slab with a stairwell opening. It stops at the inner faces of the
        # walls: a face lying in the same plane as a wall face flickers when rendered.
        st = 0.2
        slab = [
            aabb(hx1, xi1, yi0, yi1, z2 - st, z2, FLOOR),
            aabb(xi0, hx1, yi0, hy0, z2 - st, z2, FLOOR),
            aabb(xi0, hx1, hy1, yi1, z2 - st, z2, FLOOR),
        ]
        m.append(compound("first_floor_slab", slab))
        # guard rails around the stairwell (open at the top of the stairs)
        guard = rail_run(hx1 + 0.03, hy0 - 0.03, hx1 + 0.03, hy1, z2, DARK_WOOD)
        guard += rail_run(hx0, hy0 - 0.03, hx1 + 0.03, hy0 - 0.03, z2, DARK_WOOD)
        m.append(compound("stairwell_guard", guard))

        # first floor partition with a doorway into the bedroom
        part_x, bed_door_y = -4.0, 0.0
        m.append(compound("bedroom_wall", wall_along_y(part_x, 0.12, yi0, yi1, z2, z_roof, INNER_WALL,
                                                       [(bed_door_y, 1.0, z2, z2 + 2.1)])))

        # ---------------- ground floor furniture
        island_top = z1 + 0.92
        m.append(compound("kitchen_island", [
            aabb(-4.6, -2.8, -2.9, -2.1, z1, z1 + 0.88, CABINET),
            aabb(-4.7, -2.7, -3.0, -2.0, z1 + 0.88, island_top, COUNTERTOP),
        ]))
        m.append(box("dropoff_1_countertop", (0.40, 0.40, 0.004), (-3.7, -2.5, island_top + 0.002, 0, 0, 0),
                     DROPOFF, collision=False))
        notes["dropoff_1"] = f"countertop, x -3.70 y -2.50 z {island_top:.2f}"

        m.append(compound("kitchen_counter", [
            aabb(-6.5, -3.5, yi0, yi0 + 0.6, z1, z1 + 0.88, CABINET),
            aabb(-6.5, -3.5, yi0, yi0 + 0.65, z1 + 0.88, z1 + 0.92, COUNTERTOP),
        ]))
        m.append(compound("fridge", [aabb(-7.7, -6.9, yi0, yi0 + 0.7, z1, z1 + 1.8, STEEL)]))
        m.append(compound("sofa", [
            aabb(-3.9, -3.0, 0.6, 2.8, z1, z1 + 0.45, SOFA),
            aabb(-3.9, -3.7, 0.6, 2.8, z1, z1 + 0.85, SOFA),
            aabb(-3.9, -3.0, 0.6, 0.8, z1, z1 + 0.65, SOFA),
            aabb(-3.9, -3.0, 2.6, 2.8, z1, z1 + 0.65, SOFA),
        ]))
        m.append(compound("coffee_table", table_parts(-2.6, -1.9, 1.1, 2.3, z1, 0.44, DARK_WOOD)))
        m.append(box("rug", (1.8, 2.0, 0.004), (-2.1, 1.7, z1 + 0.006, 0, 0, 0), RUG, collision=False))
        m.append(compound("tv_stand", [
            aabb(-0.65, xi1, 0.9, 2.5, z1, z1 + 0.5, DARK_WOOD),
            aabb(-0.5, -0.44, 1.0, 2.4, z1 + 0.55, z1 + 1.35, BLACK),
            aabb(-0.55, -0.4, 1.6, 1.8, z1 + 0.5, z1 + 0.55, BLACK),
        ]))
        dining = table_parts(-6.2, -5.0, 0.6, 2.0, z1, 0.76, DARK_WOOD)
        for cx in (-5.9, -5.3):
            dining += chair_parts(cx, 0.3, z1, "-y", DARK_WOOD)
            dining += chair_parts(cx, 2.3, z1, "+y", DARK_WOOD)
        m.append(compound("dining_set", dining))
        m.append(compound("bookshelf", [aabb(-4.5, -3.3, yi1 - 0.35, yi1, z1, z1 + 1.8, DARK_WOOD)]))

        # ---------------- first floor furniture
        m.append(compound("bed", [
            aabb(-2.4, -0.6, 2.7, yi1, z2, z2 + 0.55, BEDDING),
            aabb(-2.4, -0.6, yi1 - 0.08, yi1, z2, z2 + 1.1, DARK_WOOD),
            aabb(-2.2, -0.8, yi1 - 0.5, yi1 - 0.12, z2 + 0.55, z2 + 0.65, (0.95, 0.95, 0.97), False),
        ]))
        m.append(compound("nightstand", [aabb(-3.0, -2.5, yi1 - 0.5, yi1, z2, z2 + 0.55, DARK_WOOD)]))
        m.append(compound("dresser", [aabb(-0.7, xi1, -2.2, -0.9, z2, z2 + 0.9, DARK_WOOD)]))
        desk = table_parts(-2.0, -0.8, yi0, yi0 + 0.6, z2, 0.75, DARK_WOOD)
        desk += chair_parts(-1.4, yi0 + 0.9, z2, "+y", DARK_WOOD)
        m.append(compound("desk", desk))
        m.append(box("dropoff_2_upstairs", (0.80, 0.80, 0.004), (-3.2, bed_door_y, z2 + 0.003, 0, 0, 0),
                     DROPOFF, collision=False))
        notes["dropoff_2"] = f"first floor, inside bedroom doorway, x -3.20 y {bed_door_y:.2f} z {z2:.2f}"

        m.append(point_light("light_ground", -4.0, 0.0, z1 + 2.5))
        m.append(point_light("light_first", -4.0, 0.0, z2 + 2.5))

    elif a.house == "solid":
        m.append(box("house_body", (D, W, z_roof - rise), (-D / 2, 0, (rise + z_roof) / 2, 0, 0, 0), WALL))
        m.append(box("door", (0.02, dw_door, 2.1), (0.01, door_y, rise + 1.05, 0, 0, 0), DOOR, collision=False))
        for i, wy in enumerate([-3.0, 0.5]):
            m.append(box(f"window_{i}", (0.02, 1.5, 1.25), (0.01, wy, rise + 1.55, 0, 0, 0), GLASS, collision=False))
    else:
        m.append(include(FUEL + a.fuel_model, "house", (a.fuel_x, a.fuel_y, rise, 0, 0, math.radians(a.fuel_yaw))))

    # gable roof over the box houses, ridge parallel to the front wall, stepped attic fill closes the gables
    if a.house in ("interior", "solid"):
        p, ov, rt = math.radians(35), 0.4, 0.15
        rise_roof = (D / 2) * math.tan(p)
        z_ridge = z_roof + rise_roof
        plane_len = (D / 2 + ov) / math.cos(p)
        xc = (ov - D / 2) / 2
        zc = (z_ridge + z_roof - ov * math.tan(p)) / 2 + rt / 2
        roof = [((plane_len, W + 2 * ov, rt), (xc, 0, zc, 0, p, 0), ROOF),
                ((plane_len, W + 2 * ov, rt), (-D - xc, 0, zc, 0, -p, 0), ROOF)]
        n = 10
        for i in range(n):
            h = rise_roof / n
            wdt = D * (1 - (i + 1) / n)
            if wdt <= 0:
                break
            roof.append(((wdt, W, h), (-D / 2, 0, z_roof + (i + 0.5) * h, 0, 0, 0), WALL))
        m.append(compound("roof", roof))

    # ----------------------------------------------------------------- landing and ramp
    m.append(box("landing", (Ld, L, rise), (Ld / 2, my(door_y), rise / 2, 0, 0, 0), WOOD))
    T = rise / math.cos(th) + 0.10          # thick enough that the underside is below ground everywhere
    ym = (y_top + y_bot) / 2
    m.append(box("ramp", (rw, slope_len, T),
                 (rw / 2, my(ym + math.sin(th) * T / 2), rise / 2 - math.cos(th) * T / 2, sgn * th, 0, 0), WOOD))

    if not a.no_railing:
        ps, post_h = 0.08, 1.0
        rail_t, rail_h, rail_zs = 0.04, 0.10, (0.45, 0.90)
        xr = rw - ps / 2
        rail = []
        nseg = max(1, math.ceil(run / 1.4))
        for i in range(nseg + 1):
            y = y_bot + 0.05 + (run - 0.05) * i / nseg
            rail.append(((ps, ps, post_h), (xr, my(y), deck_z(y) + post_h / 2, 0, 0, 0), RAIL))
        for hz in rail_zs:
            rail.append(((rail_t, slope_len, rail_h), (xr, my(ym), deck_z(ym) + hz, sgn * th, 0, 0), RAIL))
        y_far = y_top + L
        xl = Ld - ps / 2                    # landing outer edge
        rail.append(((ps, ps, post_h), (xl, my(y_far - ps / 2), rise + post_h / 2, 0, 0, 0), RAIL))
        rail.append(((ps, ps, post_h), (ps / 2 + 0.02, my(y_far - ps / 2), rise + post_h / 2, 0, 0, 0), RAIL))
        for hz in rail_zs:
            rail.append(((rail_t, L, rail_h), (xl, my(y_top + L / 2), rise + hz, 0, 0, 0), RAIL))
            rail.append(((Ld, rail_t, rail_h), (Ld / 2, my(y_far - ps / 2), rise + hz, 0, 0, 0), RAIL))
        if Ld - rw > ps:
            # close the landing edge that overhangs the ramp's outer side
            rail.append(((ps, ps, post_h), (xl, my(y_top + ps / 2), rise + post_h / 2, 0, 0, 0), RAIL))
            for hz in rail_zs:
                rail.append(((Ld - rw + ps, rail_t, rail_h), ((rw - ps + Ld) / 2, my(y_top + ps / 2), rise + hz, 0, 0, 0), RAIL))
        m.append(compound("ramp_railing", rail))

    # ----------------------------------------------------------------- car
    car_y = drv_y if a.car_y is None else a.car_y
    car_pose = (a.car_x, car_y, 0, 0, 0, math.radians(a.car_yaw))
    if a.car == "rav4":
        m.append(rav4_model("car", car_pose))
    elif a.car != "none":
        m.append(include(FUEL + CARS[a.car], "car", car_pose))
    if a.car_file:
        with open(a.car_file, "w") as f:
            f.write('<?xml version="1.0" ?>\n<!-- Toyota RAV4 (5th gen) at true scale, generated by gen_house_world.py.\n'
                    '     Origin: rear bumper center on the ground, +x toward the front. -->\n'
                    '<sdf version="1.9">' + rav4_model("car") + "\n</sdf>\n")
        print(f"wrote {a.car_file}")

    spawn_in = (-1.2, door_y, z1 + 0.10)
    spawn_landing = (Ld / 2, my(door_y), rise + 0.10)
    note_lines = "\n".join(f"       {k}: {v}" for k, v in notes.items())
    header = f"""<?xml version="1.0" ?>
<!-- APLC test world, generated by gen_house_world.py.
       rise {rise:.2f} m, ramp 1:{a.slope:g} ({math.degrees(th):.2f} deg, run {run:.2f} m), ramp width {rw:.2f} m,
       landing {Ld:.2f} deep x {L:.2f} m, ramp side {a.ramp_side}, house {a.house}, doorway {dw_door:.2f} m,
       car {a.car}{f" at x {a.car_x:.2f} y {car_y:.2f} yaw {a.car_yaw:g} deg" if a.car != "none" else " (spawned by world.launch.py)"}
{note_lines}
     Suggested robot spawns:
       inside, facing the front door: x {spawn_in[0]:.2f} y {spawn_in[1]:.2f} z {spawn_in[2]:.2f} yaw 0
       on the landing, facing down the ramp: x {spawn_landing[0]:.2f} y {spawn_landing[1]:.2f} z {spawn_landing[2]:.2f} yaw {-sgn*90:.0f} deg
     Ramp foot at y {my(y_bot):.2f}; driveway centered at y {drv_y:.2f}, x {dx0:.1f} to {dx1:.1f}. -->
<sdf version="1.9">
  <world name="house_ramp">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.80 0.95 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
"""
    with open(a.output, "w") as f:
        f.write(header + "".join(m) + "\n  </world>\n</sdf>\n")
    print(f"wrote {a.output} ({a.house} house)")
    for k, v in notes.items():
        print(f"  {k}: {v}")
    print(f"  spawn inside: ({spawn_in[0]:.2f}, {spawn_in[1]:.2f}, {spawn_in[2]:.2f}) yaw 0")


if __name__ == "__main__":
    main()