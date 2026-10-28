#!/usr/bin/env python3
"""Check that the URDF wheel layout matches the kiwi_drive kinematics.

    xacro hb_bot.urdf.xacro > /tmp/hb.urdf && python3 check_wheel_geometry.py /tmp/hb.urdf

For every rim joint it computes, in base_link:
  - the wheel position angle (should match wheel_angles_deg in kiwi_drive.yaml)
  - the axle direction (must point at the robot centre, that's the sign convention
    the kinematics and firmware assume)
"""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

EXPECTED = {'rim_left_joint': 60.0, 'rim_right_joint': -60.0, 'rim_back_joint': 180.0}
WHEEL_BASE_RADIUS = 0.052


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def main(path):
    root = ET.parse(path).getroot()
    ok = True
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name not in EXPECTED:
            continue
        assert joint.find('parent').get('link') == 'base_link', f'{name} must hang off base_link'
        origin = joint.find('origin')
        xyz = np.array([float(v) for v in origin.get('xyz').split()])
        rpy = [float(v) for v in origin.get('rpy').split()]
        axis_local = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        axis = rpy_matrix(*rpy) @ axis_local

        angle = math.degrees(math.atan2(xyz[1], xyz[0]))
        radius = math.hypot(xyz[0], xyz[1])
        inward = -xyz[:2] / radius
        dot = float(axis[:2] @ inward)
        good = (abs(((angle - EXPECTED[name]) + 180) % 360 - 180) < 1e-6
                and abs(radius - WHEEL_BASE_RADIUS) < 1e-9 and dot > 0.999 and abs(axis[2]) < 1e-9)
        ok &= good
        print(f'{name:16s} angle {angle:7.2f} deg  radius {radius:.4f} m  '
              f'axle {np.round(axis, 3)}  axle . inward = {dot:+.3f}  {"OK" if good else "MISMATCH"}')
    if not ok:
        sys.exit('wheel geometry does not match the kinematics')
    print('wheel geometry matches kiwi_drive kinematics')


if __name__ == '__main__':
    main(sys.argv[1])
