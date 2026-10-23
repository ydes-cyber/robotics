
"""Small checks for recommended demo nodes."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from finite_state_machine import FiniteStateMachine, validate_logic
from joint_state_publisher import JointStatePublisher
from urdf_parser import parse_robot_description


class FakeClient:
    def __init__(self):
        self.published = []
        self.services = {}

    def advertise(self, *_args):
        pass

    def subscribe(self, *_args):
        pass

    def advertise_service(self, name, _type_name, callback):
        self.services[name] = callback

    def publish(self, topic, message):
        self.published.append((topic, message))

    def call(self, *_args):
        return {"result": False}


class DemoTests(unittest.TestCase):
    def test_joint_driver(self):
        client = FakeClient()
        driver = JointStatePublisher(client)
        driver.running = False
        driver.robot = parse_robot_description('''<robot><link name="a"/><link name="b"/>
          <joint name="j" type="prismatic"><parent link="a"/><child link="b"/>
          <limit lower="0" upper="2"/></joint></robot>''')
        driver.positions = {"j": 0.0}
        driver.on_trajectory({"joint_names": ["j"], "points": [{"positions": [1.5]}]})
        driver.publish_state()
        self.assertEqual(client.published[-1][1]["position"], [1.5])
        self.assertTrue(driver.reset({})[1])
        self.assertEqual(driver.positions["j"], 1.0)

    def test_fsm_transition(self):
        client = FakeClient()
        fsm = FiniteStateMachine(client)
        fsm.active = False
        document = {"initial_state": "home", "states": [
            {"name": "home", "target": {"joint_names": ["j"], "positions": [0.0]},
             "epsilon": 0.01, "on_arrival": "next",
             "transitions": [{"name": "next", "next_state": "wave"}]},
            {"name": "wave", "target": {"joint_names": ["j"], "positions": [0.8]},
             "epsilon": 0.01, "transitions": []},
        ]}
        self.assertEqual(set(validate_logic(document)), {"home", "wave"})
        self.assertTrue(fsm.update_logic(document)[1])
        self.assertEqual(fsm.current, "home")
        fsm.on_joint_states({"name": ["j"], "position": [0.0]})
        self.assertEqual(fsm.current, "wave")
        trajectory = [msg for topic, msg in client.published if topic == "/joint_trajectory"]
        self.assertEqual(trajectory[-1]["points"][0]["positions"], [0.8])


if __name__ == "__main__":
    unittest.main()
