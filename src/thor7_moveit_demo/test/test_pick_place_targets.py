"""Offline regression against user TF measurements; never connects to ROS."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG = yaml.safe_load((ROOT / 'src/thor7_moveit_demo/config/pick_place.yaml').read_text())['vision_grasp_auto_sequence']['ros__parameters']
URDF = ET.parse(ROOT / 'src/thor7v3_description/urdf/thor7v3_robot.urdf').getroot()


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def fk(values):
    joints = {f'joint{i+1}': v for i, v in enumerate(values)}
    transforms = {'base_link': np.eye(4)}
    pending = list(URDF.findall('joint'))
    while 'tool0' not in transforms:
        progress = False
        for joint in list(pending):
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (rotation([0, 0, 1], rpy[2]) @
                                  rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0]))
            if joint.get('name') in joints:
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                transform[:3, :3] = transform[:3, :3] @ rotation(axis, joints[joint.get('name')])
            transforms[joint.find('child').get('link')] = transforms[parent] @ transform
            pending.remove(joint)
            progress = True
        if not progress:
            raise AssertionError('tool0 has no chain to base_link')
    return transforms['tool0']


class TaughtTargets(unittest.TestCase):
    def test_joint_name_mapping_matches_home_and_place_feedback(self):
        # Independent TF observations supplied by the user, not generated from YAML.
        for name, expected in [('home_joints', [-.156891057, .328941476, .308893801]),
                               ('place_joints', [.113042648, .715647067, .215062180])]:
            actual = fk(CONFIG[name])
            error = np.linalg.norm(actual[:3, 3] - expected)
            print(name, 'FK xyz=', actual[:3, 3], 'measurement error mm=', error*1000)
            self.assertLess(error, .0003)
            self.assertGreater(np.dot(actual[:3, 2], [0, 0, -1]), math.cos(math.radians(.1)))

    def test_keeping_grasp_j6_preserves_place_position_and_down_axis(self):
        joints = list(CONFIG['place_joints'])
        taught = fk(joints)
        for j6 in [-3.0, -.8, 0., .7, 2., 3.5, 5.0]:
            joints[5] = j6
            actual = fk(joints)
            np.testing.assert_allclose(actual[:3, 3], taught[:3, 3], atol=1e-9)
            np.testing.assert_allclose(actual[:3, 2], taught[:3, 2], atol=1e-9)

    def test_home_winding_is_preserved(self):
        self.assertGreater(CONFIG['home_joints'][5], math.pi)
        self.assertLess(CONFIG['home_joints'][5], 2*math.pi)


if __name__ == '__main__':
    unittest.main()
