"""Focused offline checks for parsing, FK, and generic hub routing."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from forward_kinematics import joint_transform, tf_message, world_transforms
from hub import Hub
from transforms import (axis_angle, compose, flatten_column_major,
                        matrix_to_quaternion, rpy_rotation, translation)
from urdf_parser import parse_robot_description

FETCH = Path(__file__).resolve().parents[3] / "robots/fetch/fetch.urdf"


class FakePeer:
    def __init__(self):
        self.messages = []
        self.closed = False

    def send(self, message):
        self.messages.append(message)
        return True

    def close(self):
        self.closed = True


class ParserTests(unittest.TestCase):
    def test_fetch_and_defaults(self):
        if FETCH.exists():  # the submission archive contains only starter/python
            fetch = parse_robot_description(FETCH.read_text(encoding="utf-8"))
            self.assertEqual((len(fetch.links), len(fetch.joints), fetch.root), (26, 25, "base_link"))
            self.assertEqual(len(tf_message(fetch, {}, {"sec": 0, "nanosec": 0})["transforms"]), 25)
            self.assertEqual(len(world_transforms(fetch, {})), 26)
        robot = parse_robot_description('<robot xmlns="urn:urdf"><joint name="j" type="fixed">'
                                        '<parent link="a"/><child link="b"/><axis xyz="0 0 0"/>'
                                        '</joint><link name="b"/><link name="a"/></robot>')
        self.assertEqual(robot.root, "a")
        self.assertEqual(robot.joints[0].axis, (0.0, 0.0, 0.0))

    def test_numbers_visuals_and_nested_joint(self):
        text = '''<?xml version="1.0"?><robot><gazebo><joint name="fake"/></gazebo>
          <link name="child"><visual><geometry><box size=" .25\t+1 3. "/></geometry></visual></link>
          <link name="base"/><joint name="j" type="revolute"><parent link="base"/>
          <child link="child"/><origin xyz="1e-400 0 -2E+0"/>
          <limit lower="-3.14" upper="3.14"/></joint></robot>'''
        robot = parse_robot_description(text)
        self.assertEqual(len(robot.joints), 1)
        self.assertEqual(robot.joints[0].xyz, (0.0, 0.0, -2.0))
        self.assertEqual(robot.joints[0].axis, (1.0, 0.0, 0.0))

    def test_rejection_does_not_look_inside_unknown_elements(self):
        invalid = [
            "", "<x/>", "<robot/>", '<robot><link/></robot>',
            '<robot><link name="a"/><joint name="j" type="fixed"/></robot>',
            '<robot><link name="a"/><link name="b"/></robot>',
            '<robot><link name="a"><visual/></link></robot>',
            '<robot><link name="a"/><link name="b"/><joint name="j" type="revolute">'
            '<parent link="a"/><child link="b"/></joint></robot>',
            '<robot><link name="a"/><link name="b"/><joint name="j" type="fixed">'
            '<parent link="a"/><child link="b"/><axis xyz="1 2"/></joint></robot>',
            '<robot><link name="a"/><link name="b"/><joint name="j" type="planar">'
            '<parent link="a"/><child link="b"/></joint></robot>',
            '<robot><link name="a"/><joint name="j" type="fixed">'
            '<parent link="a"/><child link="missing"/></joint></robot>',
            '<robot><link name="a"/><link name="b"/><joint name="j" type="prismatic">'
            '<parent link="a"/><child link="b"/><limit lower="two"/></joint></robot>',
            '<robot><link name="a"><visual><geometry><capsule/></geometry></visual></link></robot>',
        ]
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_robot_description(text)


class MathTests(unittest.TestCase):
    def test_rpy_axis_angle_and_column_major(self):
        matrix = compose(translation((1, 2, 3)), rpy_rotation((0, 0, math.pi/2)))
        self.assertAlmostEqual(matrix[0][1], -1)
        self.assertAlmostEqual(matrix[1][0], 1)
        self.assertEqual(flatten_column_major(matrix)[12:15], [1.0, 2.0, 3.0])
        q = matrix_to_quaternion(axis_angle((0, 2, 0), math.pi))
        self.assertAlmostEqual(sum(value*value for value in q), 1.0)
        self.assertAlmostEqual(abs(q[1]), 1.0)
        roll = rpy_rotation((math.pi/2, 0, 0))
        pitch = rpy_rotation((0, math.pi/2, 0))
        yaw = rpy_rotation((0, 0, math.pi/2))
        self.assertAlmostEqual(roll[2][1], 1.0)
        self.assertAlmostEqual(pitch[0][2], 1.0)
        self.assertAlmostEqual(yaw[1][0], 1.0)
        combined = rpy_rotation((0.2, -0.3, 0.4))
        by_axes = compose(compose(rpy_rotation((0, 0, 0.4)),
                                  rpy_rotation((0, -0.3, 0))), rpy_rotation((0.2, 0, 0)))
        for row in range(3):
            for col in range(3):
                self.assertAlmostEqual(combined[row][col], by_axes[row][col])

    def test_motion_after_origin(self):
        robot = parse_robot_description('''<robot><link name="base"/><link name="tip"/>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="tip"/>
          <origin rpy="0 0 1.5707963267948966"/><axis xyz="2 0 0"/><limit/></joint>
          </robot>''')
        matrix = joint_transform(robot.joints[0], 0.7)
        self.assertAlmostEqual(matrix[0][3], 0.0, places=10)
        self.assertAlmostEqual(matrix[1][3], 1.4)
        world = world_transforms(robot, {"slide": 0.7})
        self.assertEqual(set(world), {"base", "tip"})
        self.assertEqual(len(tf_message(robot, {}, {"sec": 0, "nanosec": 0})["transforms"]), 1)


class HubTests(unittest.TestCase):
    def test_service_correlation_and_large_reply(self):
        hub, provider, caller = Hub(), FakePeer(), FakePeer()
        hub.dispatch(provider, {"op": "advertise_service", "service": "/s", "type": "T"})
        hub.dispatch(caller, {"op": "call_service", "service": "/s", "id": ["opaque", 1],
                              "args": {"value": "x" * 1_048_000}})
        forwarded = provider.messages[-1]
        self.assertEqual(len(forwarded["args"]["value"]), 1_048_000)
        impostor = FakePeer()
        hub.dispatch(impostor, {"op": "service_response", "service": "/s",
                                "id": forwarded["id"], "values": {}, "result": True})
        self.assertEqual(caller.messages, [])
        hub.dispatch(provider, {"op": "service_response", "service": "/s",
                                "id": forwarded["id"], "values": {"value": "y" * 1_048_000},
                                "result": True, "status": ""})
        self.assertEqual(caller.messages[-1]["id"], ["opaque", 1])
        self.assertEqual(len(caller.messages[-1]["values"]["value"]), 1_048_000)

    def test_topic_idempotency_and_disconnect(self):
        hub, publisher, reader = Hub(), FakePeer(), FakePeer()
        subscribe = {"op": "subscribe", "topic": "/tf", "type": "tf/tfMessage"}
        hub.dispatch(reader, subscribe)
        hub.dispatch(reader, subscribe)
        hub.dispatch(publisher, {"op": "publish", "topic": "/tf", "msg": {"transforms": []}})
        self.assertEqual(sum(item["op"] == "publish" for item in reader.messages), 1)
        hub.disconnect(reader)
        self.assertTrue(reader.closed)


if __name__ == "__main__":
    unittest.main()
