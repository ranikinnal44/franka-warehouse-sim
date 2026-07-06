#!/usr/bin/env python3
# Copyright (c) 2026 Franka test workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Generate a warehouse world SDF: a table with the arm mounted on it and a
single box of the configured size resting on the tabletop in front of the arm.

The warehouse shell (floor, four walls, lighting, system plugins) is fixed. A
static table stands at the origin; the arm's ``link0`` is fixed to the world at
the tabletop height (set in the launch via the ``xyz`` xacro arg, so it is
mounted on the table). One box sits on the tabletop ``box_x`` metres in front.

Box dimensions are given in the same X Y Z order as the request
(200x300x400 mm -> 0.2 0.3 0.4 m). The box is a dynamic rigid body by default
(so it can be grasped); pass --static-box to pin it in place.
"""

import argparse

# Cardboard-ish packed density (kg/m^3); used for the dynamic box inertia.
DENSITY = 100.0


def box_model(name, size, pose, static, color):
    dx, dy, dz = size
    r, g, b = color
    if static:
        inertial = ''
        static_tag = '      <static>true</static>\n'
    else:
        mass = DENSITY * dx * dy * dz
        ixx = mass / 12.0 * (dy * dy + dz * dz)
        iyy = mass / 12.0 * (dx * dx + dz * dz)
        izz = mass / 12.0 * (dx * dx + dy * dy)
        inertial = (
            '        <inertial>\n'
            f'          <mass>{mass:.4f}</mass>\n'
            '          <inertia>\n'
            f'            <ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>\n'
            f'            <iyy>{iyy:.6f}</iyy><iyz>0</iyz>\n'
            f'            <izz>{izz:.6f}</izz>\n'
            '          </inertia>\n'
            '        </inertial>\n'
        )
        static_tag = ''
    return (
        f'    <model name="{name}">\n'
        f'{static_tag}'
        f'      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>\n'
        '      <link name="link">\n'
        f'{inertial}'
        '        <collision name="collision">\n'
        '          <geometry>\n'
        f'            <box><size>{dx:.4f} {dy:.4f} {dz:.4f}</size></box>\n'
        '          </geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry>\n'
        f'            <box><size>{dx:.4f} {dy:.4f} {dz:.4f}</size></box>\n'
        '          </geometry>\n'
        '          <material>\n'
        f'            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>\n'
        f'            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>\n'
        '            <specular>0.1 0.1 0.1 1</specular>\n'
        '          </material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    )


def _box_link(name, size, pose, color):
    dx, dy, dz = size
    r, g, b = color
    return (
        f'      <link name="{name}">\n'
        '        <collision name="c">\n'
        f'          <geometry><box><size>{dx:.4f} {dy:.4f} {dz:.4f}</size></box></geometry>\n'
        f'          <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>\n'
        '        </collision>\n'
        '        <visual name="v">\n'
        f'          <geometry><box><size>{dx:.4f} {dy:.4f} {dz:.4f}</size></box></geometry>\n'
        f'          <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>\n'
        '          <material>\n'
        f'            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>\n'
        f'            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>\n'
        '          </material>\n'
        '        </visual>\n'
        '      </link>\n'
    )


def table_model(top_z, size_x, size_y, center_x, thickness, leg):
    """A static four-legged table whose top surface is at ``top_z``.

    The arm mounts on this surface and the box rests on it.
    """
    wood = (0.55, 0.38, 0.22)
    top_center_z = top_z - thickness / 2.0
    leg_h = top_z - thickness
    leg_cz = leg_h / 2.0
    hx = size_x / 2.0 - leg / 2.0
    hy = size_y / 2.0 - leg / 2.0

    links = _box_link('top', (size_x, size_y, thickness),
                      (center_x, 0.0, top_center_z), wood)
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            links += _box_link(f'leg_{idx}', (leg, leg, leg_h),
                               (center_x + sx * hx, sy * hy, leg_cz), wood)
            idx += 1
    return f'    <model name="table">\n      <static>true</static>\n{links}    </model>\n'


WAREHOUSE_SHELL = """    <!-- ================= PHYSICS ================= -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <!-- ================= SYSTEM PLUGINS ================= -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- ================= LIGHTING ================= -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light type="point" name="ceiling_0">
      <pose>0 0 3.8 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>20</range><constant>0.5</constant>
        <linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <light type="point" name="ceiling_1">
      <pose>3 3 3.8 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>20</range><constant>0.5</constant>
        <linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <!-- ================= WAREHOUSE SHELL ================= -->
    <!-- 12 x 10 m enclosed room, 4 m tall. Arm mounts on the table at origin. -->
    <model name="warehouse">
      <static>true</static>
      <!-- Floor -->
      <link name="floor">
        <collision name="c">
          <geometry><box><size>12 10 0.1</size></box></geometry>
          <pose>0 0 -0.05 0 0 0</pose>
        </collision>
        <visual name="v">
          <geometry><box><size>12 10 0.1</size></box></geometry>
          <pose>0 0 -0.05 0 0 0</pose>
          <material>
            <ambient>0.4 0.4 0.42 1</ambient>
            <diffuse>0.5 0.5 0.52 1</diffuse>
          </material>
        </visual>
      </link>
      <!-- +X wall -->
      <link name="wall_px">
        <collision name="c"><geometry><box><size>0.1 10 4</size></box></geometry>
          <pose>6 0 2 0 0 0</pose></collision>
        <visual name="v"><geometry><box><size>0.1 10 4</size></box></geometry>
          <pose>6 0 2 0 0 0</pose>
          <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse></material></visual>
      </link>
      <!-- -X wall -->
      <link name="wall_nx">
        <collision name="c"><geometry><box><size>0.1 10 4</size></box></geometry>
          <pose>-6 0 2 0 0 0</pose></collision>
        <visual name="v"><geometry><box><size>0.1 10 4</size></box></geometry>
          <pose>-6 0 2 0 0 0</pose>
          <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse></material></visual>
      </link>
      <!-- +Y wall -->
      <link name="wall_py">
        <collision name="c"><geometry><box><size>12 0.1 4</size></box></geometry>
          <pose>0 5 2 0 0 0</pose></collision>
        <visual name="v"><geometry><box><size>12 0.1 4</size></box></geometry>
          <pose>0 5 2 0 0 0</pose>
          <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse></material></visual>
      </link>
      <!-- -Y wall -->
      <link name="wall_ny">
        <collision name="c"><geometry><box><size>12 0.1 4</size></box></geometry>
          <pose>0 -5 2 0 0 0</pose></collision>
        <visual name="v"><geometry><box><size>12 0.1 4</size></box></geometry>
          <pose>0 -5 2 0 0 0</pose>
          <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse></material></visual>
      </link>
    </model>
"""


def generate(world_name, size, table, box_xy, static_box):
    dims_mm = 'x'.join(str(int(round(s * 1000))) for s in size)
    table_sdf = table_model(table['height'], table['size_x'], table['size_y'],
                            table['center_x'], table['thickness'], table['leg'])
    # Box rests on the tabletop at (box_x, box_y), bottom on the surface.
    box_x, box_y = box_xy
    box_pose = (box_x, box_y, table['height'] + size[2] / 2.0)
    box = box_model('box', size, box_pose, static=static_box, color=(0.85, 0.35, 0.12))

    header = (
        '<?xml version="1.0" ?>\n'
        '<!-- Generated by generate_world.py from config/scene.yaml. Do not edit by hand.\n'
        f'     Table (top at {table["height"]} m) with the arm mounted on it and a\n'
        f'     single {dims_mm} mm box on the tabletop at ({box_x}, {box_y}) m. -->\n'
        '<sdf version="1.6">\n'
        f'  <world name="{world_name}">\n'
    )
    footer = '  </world>\n</sdf>\n'
    body = (WAREHOUSE_SHELL
            + '\n    <!-- ================= TABLE ================= -->\n' + table_sdf
            + '\n    <!-- ================= BOX ================= -->\n' + box)
    return header + body + footer


def main():
    import yaml

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True, help='path to scene.yaml')
    p.add_argument('--world', required=True, help='world key in scene.yaml (small/large)')
    p.add_argument('--static-box', action='store_true',
                   help='pin the box in place instead of a graspable rigid body')
    p.add_argument('--output', required=True, help='output .sdf path')
    args = p.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if args.world not in cfg['worlds']:
        raise SystemExit(f"world '{args.world}' not in {list(cfg['worlds'])}")
    w = cfg['worlds'][args.world]

    sdf = generate(w['world_name'], w['box_size'], cfg['table'], w['box_xy'],
                   static_box=args.static_box)
    with open(args.output, 'w') as f:
        f.write(sdf)
    print(f"Wrote {args.output} (table top {cfg['table']['height']} m, "
          f"{'static' if args.static_box else 'dynamic'} box at {w['box_xy']})")


if __name__ == '__main__':
    main()
