#!/usr/bin/env python
"""Each upper-arm joint must use the same limits as its lower-arm twin."""
import os
import unittest
import xml.etree.ElementTree as ET

MACRO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'urdf', 'duaro_macro.xacro')


def joint_limits():
    limits = {}
    for joint in ET.parse(MACRO).getroot().iter('joint'):
        limit = joint.find('limit')
        if limit is not None:
            limits[joint.get('name').replace('${prefix}', '')] = limit.attrib
    return limits


class TestDuaroJointLimits(unittest.TestCase):
    def test_upper_joints_match_lower_joints(self):
        limits = joint_limits()
        for n in range(1, 5):
            lower = limits['lower_joint%d' % n]
            upper = limits['upper_joint%d' % n]
            self.assertEqual(upper['velocity'].replace('UPPER', 'LOWER'), lower['velocity'],
                             'upper_joint%d velocity limit' % n)


if __name__ == '__main__':
    unittest.main()
