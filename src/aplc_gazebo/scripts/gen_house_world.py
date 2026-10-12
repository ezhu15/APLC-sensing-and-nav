#!/usr/bin/env python3
"""Generate a Gazebo Harmonic world: house raised on a foundation, a straight
wooden ramp running along the front wall down from a landing at the door,
a concrete apron, an asphalt driveway, a street, and a parked car.

Layout (default, all in meters, z up):
  - House front wall lies on the plane x = 0; the house extends toward -x.
  - The house floor (and the door threshold) sits at z = --rise.
  - Landing: in front of the door, against the wall, top at z = rise.
  - Ramp: runs along the wall from the landing down to the apron
    (toward -y with --ramp-side right, toward +y with --ramp-side left).
  - Apron (concrete) along the front of the house, driveway (asphalt) heading
    out in +x from the end of the ramp, street across the end of the driveway.

Example:
  python3 gen_house_world.py -o house_ramp.sdf
  python3 gen_house_world.py --rise 0.6 --slope 15 -o house_ramp_tall.sdf
  python3 gen_house_world.py --house fuel --fuel-x -6 --fuel-y 0 -o house_fuel.sdf
"""
import argparse
import math

# ----------------------------------------------------------------------------- colors
GRASS = (0.30, 0.52, 0.24)
CONCRETE = (0.70, 0.70, 0.67)
ASPHALT = (0.20, 0.20, 0.22)
STREET = (0.13, 0.13, 0.14)
WALL = (0.86, 0.85, 0.81)
BRICK = (0.55, 0.26, 0.20)
ROOF = (0.24, 0.24, 0.26)
DOOR = (0.36, 0.20, 0.12)
GLASS = (0.22, 0.32, 0.42)
WOOD = (0.56, 0.33, 0.17)
RAIL = (0.50, 0.29, 0.14)

FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
CARS = {"suv": "SUV", "pickup": "Pickup", "hatchback": "Hatchback", "prius": "Prius Hybrid"}


def box(name, size, pose, color, collision=True):
    """A static box model. pose = (x, y, z, roll, pitch, yaw), SDF convention."""
    sx, sy, sz = size
    x, y, z, r, p, yw = pose
    c = "{:.3f} {:.3f} {:.3f} 1".format(*color)
    geom = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    col = f'\n        <collision name="collision">{geom}</collision>' if collision else ""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {r:.5f} {p:.5f} {yw:.5f}</pose>
      <link name="link">{col}
        <visual name="visual">{geom}
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>"""


def include(uri, name, pose):
    x, y, z, r, p, yw = pose
    return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {r:.5f} {p:.5f} {yw:.5f}</pose>
    </include>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", default="house_ramp.sdf")
    # ramp and landing
    ap.add_argument("--rise", type=float, default=0.45, help="door threshold / landing height (m)")
    ap.add_argument("--slope", type=float, default=12.0, help="horizontal run per unit rise (12 = 1:12)")
    ap.add_argument("--ramp-width", type=float, default=1.2, help="ramp and landing width out from the wall (m)")
    ap.add_argument("--landing-length", type=float, default=1.5, help="landing length along the wall (m)")
    ap.add_argument("--ramp-side", choices=["right", "left"], default="right",
                    help="direction the ramp descends from the door, seen facing the house")
    ap.add_argument("--no-railing", action="store_true")
    # house
    ap.add_argument("--house", choices=["primitive", "fuel"], default="primitive")
    ap.add_argument("--house-width", type=float, default=10.0, help="along the front wall (m)")
    ap.add_argument("--house-depth", type=float, default=8.0)
    ap.add_argument("--wall-height", type=float, default=2.7)
    ap.add_argument("--door-y", type=float, default=3.5, help="door center along the wall (m)")
    ap.add_argument("--fuel-model", default="House 1")
    ap.add_argument("--fuel-x", type=float, default=-5.0)
    ap.add_argument("--fuel-y", type=float, default=0.0)
    ap.add_argument("--fuel-yaw", type=float, default=0.0, help="degrees")
    # site
    ap.add_argument("--apron-width", type=float, default=2.0)
    ap.add_argument("--driveway-length", type=float, default=12.0)
    ap.add_argument("--driveway-width", type=float, default=3.5)
    ap.add_argument("--car", choices=list(CARS) + ["none"], default="suv")
    ap.add_argument("--car-yaw", type=float, default=0.0, help="degrees")
    a = ap.parse_args()

    rise, W, D = a.rise, a.house_width, a.house_depth
    rw, L = a.ramp_width, a.landing_length
    run = rise * a.slope
    th = math.atan2(rise, run)
    slope_len = math.hypot(run, rise)
    sgn = 1.0 if a.ramp_side == "right" else -1.0  # right: ramp descends toward -y

    # Ramp geometry is built in a frame where the ramp descends toward -y,
    # then mirrored about the door for --ramp-side left.
    def my(y):
        return a.door_y + sgn * (y - a.door_y)

    y_top = a.door_y - L / 2          # where landing meets ramp
    y_bot = y_top - run               # where ramp meets the ground

    def deck_z(y):
        return rise * (y - y_bot) / run

    m = []

    # ----------------------------------------------------------------- ground and site
    m.append(f"""
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material><ambient>{GRASS[0]} {GRASS[1]} {GRASS[2]} 1</ambient><diffuse>{GRASS[0]} {GRASS[1]} {GRASS[2]} 1</diffuse></material>
        </visual>
      </link>
    </model>""")

    # Paved surfaces are visual-only and 4 mm thick so there is no lip to drive over;
    # the ground plane carries the collision for them.
    ys_ramp = sorted([my(y_bot), my(y_top + L)])
    ap_y0 = min(-W / 2, ys_ramp[0]) - 0.5
    ap_y1 = max(W / 2, ys_ramp[1]) + 0.5
    pave_t, pave_z = 0.004, 0.002
    m.append(box("apron", (a.apron_width, ap_y1 - ap_y0, pave_t),
                 (a.apron_width / 2, (ap_y0 + ap_y1) / 2, pave_z, 0, 0, 0), CONCRETE, collision=False))

    dw_y = my(y_bot) - sgn * (a.driveway_width / 2 - 1.0)   # ramp foot sits 1 m in from the driveway's house-side edge
    dw_x0, dw_x1 = a.apron_width, a.apron_width + a.driveway_length
    m.append(box("driveway", (a.driveway_length, a.driveway_width, pave_t),
                 ((dw_x0 + dw_x1) / 2, dw_y, pave_z, 0, 0, 0), ASPHALT, collision=False))
    m.append(box("street", (7.0, 40.0, pave_t), (dw_x1 + 3.5, 0, pave_z, 0, 0, 0), STREET, collision=False))

    # ----------------------------------------------------------------- house
    # Foundation fills the gap between the ground and the raised floor in both modes.
    m.append(box("foundation", (D, W, rise), (-D / 2, 0, rise / 2, 0, 0, 0), BRICK))

    if a.house == "primitive":
        z0, z1 = rise, rise + a.wall_height
        m.append(box("house_body", (D, W, a.wall_height), (-D / 2, 0, (z0 + z1) / 2, 0, 0, 0), WALL))
        # door and windows: thin visual-only panels on the front wall
        m.append(box("door", (0.02, 0.95, 2.1), (0.01, a.door_y, z0 + 1.05, 0, 0, 0), DOOR, collision=False))
        for i, wy in enumerate([-3.0, 0.5]):
            m.append(box(f"window_{i}", (0.02, 1.5, 1.25), (0.01, wy, z0 + 1.55, 0, 0, 0), GLASS, collision=False))
        # gable roof, ridge parallel to the front wall
        p, ov, rt = math.radians(35), 0.4, 0.15
        rise_roof = (D / 2) * math.tan(p)
        z_ridge = z1 + rise_roof
        plane_len = (D / 2 + ov) / math.cos(p)
        xc = (ov - D / 2) / 2
        zc = (z_ridge + z1 - ov * math.tan(p)) / 2 + rt / 2
        m.append(box("roof_front", (plane_len, W + 2 * ov, rt), (xc, 0, zc, 0, p, 0), ROOF))
        m.append(box("roof_back", (plane_len, W + 2 * ov, rt), (-D - xc, 0, zc, 0, -p, 0), ROOF))
        # attic fill under the roof (stepped), closes the gable ends
        n = 10
        for i in range(n):
            h = rise_roof / n
            wdt = D * (1 - (i + 1) / n)
            if wdt <= 0:
                break
            m.append(box(f"attic_{i}", (wdt, W, h), (-D / 2, 0, z1 + (i + 0.5) * h, 0, 0, 0), WALL))
    else:
        m.append(include(FUEL + a.fuel_model, "house",
                         (a.fuel_x, a.fuel_y, rise, 0, 0, math.radians(a.fuel_yaw))))

    # ----------------------------------------------------------------- landing and ramp
    m.append(box("landing", (rw, L, rise), (rw / 2, my(a.door_y), rise / 2, 0, 0, 0), WOOD))

    # The ramp is one tilted box thick enough that its underside is below ground
    # everywhere: above ground it is an exact solid wedge, matching a boarded-in ramp.
    T = rise / math.cos(th) + 0.10
    ym = (y_top + y_bot) / 2
    cy = ym + math.sin(th) * T / 2
    cz = rise / 2 - math.cos(th) * T / 2
    m.append(box("ramp", (rw, slope_len, T), (rw / 2, my(cy), cz, sgn * th, 0, 0), WOOD))

    # ----------------------------------------------------------------- railing
    if not a.no_railing:
        ps, post_h = 0.08, 1.0
        rail_t, rail_h, rail_zs = 0.04, 0.10, (0.45, 0.90)
        xr = rw - ps / 2                      # outer edge of ramp and landing

        # posts along the ramp
        nseg = max(1, math.ceil(run / 1.4))
        for i in range(nseg + 1):
            y = y_bot + 0.05 + (run - 0.05) * i / nseg
            m.append(box(f"ramp_post_{i}", (ps, ps, post_h), (xr, my(y), deck_z(y) + post_h / 2, 0, 0, 0), RAIL))
        for j, hz in enumerate(rail_zs):
            m.append(box(f"ramp_rail_{j}", (rail_t, slope_len, rail_h),
                         (xr, my(ym), deck_z(ym) + hz, sgn * th, 0, 0), RAIL))

        # landing: outer side and the far end back to the wall
        y_far = y_top + L
        for k, y in enumerate([y_far - ps / 2]):
            m.append(box(f"landing_post_{k}", (ps, ps, post_h), (xr, my(y), rise + post_h / 2, 0, 0, 0), RAIL))
        m.append(box("landing_post_wall", (ps, ps, post_h), (ps / 2 + 0.02, my(y_far - ps / 2), rise + post_h / 2, 0, 0, 0), RAIL))
        for j, hz in enumerate(rail_zs):
            m.append(box(f"landing_side_rail_{j}", (rail_t, L, rail_h), (xr, my(y_top + L / 2), rise + hz, 0, 0, 0), RAIL))
            m.append(box(f"landing_end_rail_{j}", (rw, rail_t, rail_h), (rw / 2, my(y_far - ps / 2), rise + hz, 0, 0, 0), RAIL))

    # ----------------------------------------------------------------- car
    if a.car != "none":
        car_x = dw_x0 + 0.6 * a.driveway_length
        m.append(include(FUEL + CARS[a.car], "car", (car_x, dw_y, 0, 0, 0, math.radians(a.car_yaw))))

    spawn = (rw / 2, my(a.door_y), rise + 0.10)
    header = f"""<?xml version="1.0" ?>
<!-- APLC test world: house with raised floor, landing and straight ramp to the door.
     Generated by gen_house_world.py. Parameters:
       rise {rise:.2f} m, slope 1:{a.slope:g} ({math.degrees(th):.2f} deg), ramp run {run:.2f} m,
       ramp width {rw:.2f} m, landing {rw:.2f} x {L:.2f} m, ramp side {a.ramp_side}, house {a.house}, car {a.car}
     Suggested robot spawn (on the landing, facing the ramp): x {spawn[0]:.2f}  y {spawn[1]:.2f}  z {spawn[2]:.2f}  yaw {-sgn * 90:.0f} deg
     Ramp foot at y {my(y_bot):.2f}; driveway centered at y {dw_y:.2f}, x {dw_x0:.1f} to {dw_x1:.1f}. -->
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
    print(f"wrote {a.output}: rise {rise} m, run {run:.2f} m, {math.degrees(th):.2f} deg, "
          f"spawn ({spawn[0]:.2f}, {spawn[1]:.2f}, {spawn[2]:.2f})")


if __name__ == "__main__":
    main()
