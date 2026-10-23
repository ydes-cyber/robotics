"""Full gateway-to-node contract check using chunked in-memory byte streams."""
import itertools
import json
import sys
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hub import Hub, Peer
from node_client import NodeClient
from robot_state import RobotStatePublisher
from rosbridge import Gateway
from services import ParameterServer
from world_state import RobotWorldStatePublisher

ARM = '''<robot><link name="tip"/><link name="base"/>
<joint name="j" type="revolute"><parent link="base"/><child link="tip"/>
<origin xyz="1 0 0"/><axis xyz="0 0 2"/><limit lower="-4" upper="4"/>
</joint></robot>'''
OTHER = '''<robot><link name="root"/><link name="new_tip"/>
<joint name="fixed" type="fixed"><parent link="root"/><child link="new_tip"/>
<origin xyz="0 2 0"/></joint></robot>'''


class MemorySocket:
    """Byte stream that splits writes and supports complete-line reads."""
    def __init__(self):
        self.chunks = deque()
        self.buffer = bytearray()
        self.condition = threading.Condition()
        self.partner = None
        self.closed = False
        self.remote_closed = False
        self.timeout = None

    @classmethod
    def pair(cls):
        left, right = cls(), cls()
        left.partner, right.partner = right, left
        return left, right

    def sendall(self, payload):
        for start in range(0, len(payload), 4096):
            with self.partner.condition:
                if self.closed or self.partner.closed:
                    raise OSError("closed socket")
                self.partner.chunks.append(payload[start:start + 4096])
                self.partner.condition.notify()

    def readline(self, limit=None):
        deadline = time.monotonic() + self.timeout if self.timeout else None
        with self.condition:
            while True:
                newline = self.buffer.find(b"\n")
                if newline >= 0:
                    length = min(newline + 1, limit or newline + 1)
                    result = bytes(self.buffer[:length])
                    del self.buffer[:length]
                    return result
                if limit and len(self.buffer) >= limit:
                    result = bytes(self.buffer[:limit])
                    del self.buffer[:limit]
                    return result
                if self.chunks:
                    self.buffer.extend(self.chunks.popleft())
                    continue
                if self.remote_closed:
                    return b""
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise TimeoutError("line read timed out")
                self.condition.wait(remaining)

    def makefile(self, _mode):
        return self

    def settimeout(self, value):
        self.timeout = value

    def shutdown(self, _how):
        self.close()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        if self.partner:
            with self.partner.condition:
                self.partner.remote_closed = True
                self.partner.condition.notify_all()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class External:
    ids = itertools.count()

    def __init__(self, sock):
        self.sock = sock
        self.stream = sock.makefile("rb")

    def send(self, message):
        self.sock.sendall((json.dumps(message) + "\n").encode("utf-8"))

    def receive(self, predicate, seconds=3.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.sock.settimeout(max(0.1, deadline - time.monotonic()))
            line = self.stream.readline()
            if not line:
                raise AssertionError("socket closed before expected message")
            message = json.loads(line)
            if predicate(message):
                return message
        raise AssertionError("expected message did not arrive")

    def call(self, service, args):
        call_id = f"external-{next(self.ids)}"
        self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
        return self.receive(lambda msg: msg.get("op") == "service_response" and
                            msg.get("id") == call_id)

    def close(self):
        self.sock.close()
        self.stream.close()


class RuntimeTests(unittest.TestCase):
    def test_transport_description_fk_reload_and_big_lines(self):
        gateway = Gateway.__new__(Gateway)
        gateway.hub = Hub()
        gateway.running = True
        clients = []

        def connect(_address=None):
            server, client = MemorySocket.pair()
            peer = Peer(server, gateway.hub.disconnect)
            threading.Thread(target=gateway._client, args=(peer,), daemon=True).start()
            clients.append(client)
            return client

        with patch("socket.create_connection", side_effect=connect):
            param_client = NodeClient()
            ParameterServer(param_client)
            robot_client = NodeClient()
            robot = RobotStatePublisher(robot_client)
            world_client = NodeClient()
            world = RobotWorldStatePublisher(world_client)
        service = External(connect())
        tf = External(connect())
        xform = External(connect())
        try:
            tf.send({"op": "subscribe", "topic": "/tf", "type": "tf/tfMessage"})
            xform.send({"op": "subscribe", "topic": "/xform_world",
                        "type": "xform_world/MatrixTransformArray"})
            self.assertEqual(tf.receive(lambda m: m.get("op") == "status")["level"], "info")
            self.assertEqual(xform.receive(lambda m: m.get("op") == "status")["level"], "info")

            # The JSON request and the get_param reply each exceed one MiB.
            large = ARM.replace("</robot>", "<!--" + "x" * 1_048_550 + "--></robot>")
            set_reply = service.call("/param_server/set_param",
                                     {"name": "robot_description", "value": large})
            self.assertTrue(set_reply["result"])
            version = set_reply["values"]["version"]
            self.assertEqual(service.call("/param_server/get_param",
                                          {"name": "robot_description"})["values"]["value"], large)
            status = self.wait_status(service, version)
            self.assertTrue(status["accepted"])
            self.assertEqual(status["root_link"], "base")
            first_tf = tf.receive(lambda m: m.get("op") == "publish" and
                                  m.get("msg", {}).get("transforms"))
            self.assertEqual(len(first_tf["msg"]["transforms"]), 1)
            first_world = xform.receive(lambda m: m.get("op") == "publish" and
                                        {e["child_frame_id"] for e in m.get("msg", {}).get("transforms", [])}
                                        == {"base", "tip"})
            self.assertEqual(len(first_world["msg"]["transforms"]), 2)

            late = External(connect())
            late.send({"op": "subscribe", "topic": "/tf", "type": "tf/tfMessage"})
            late.receive(lambda m: m.get("op") == "status")
            self.assertEqual(len(late.receive(lambda m: m.get("op") == "publish",
                                              seconds=0.5)["msg"]["transforms"]), 1)
            late.close()

            joint = External(connect())
            joint.send({"op": "advertise", "topic": "/joint_states", "type": "sensor_msgs/JointState"})
            joint.send({"op": "publish", "topic": "/joint_states",
                        "msg": {"header": {"stamp": {"sec": 7, "nanosec": 9}},
                                "name": ["j"], "position": [1.0]}})
            moving = tf.receive(lambda m: m.get("op") == "publish" and
                                m["msg"]["transforms"][0]["header"]["stamp"]["sec"] == 7)
            rotation = moving["msg"]["transforms"][0]["transform"]["rotation"]
            self.assertAlmostEqual(rotation["z"], __import__("math").sin(0.5), places=5)

            joint.send({"op": "publish", "topic": "/joint_states",
                        "msg": {"header": {"stamp": {"sec": 8, "nanosec": 10}},
                                "name": ["unknown"], "position": [9.0]}})
            reset_tf = tf.receive(lambda m: m.get("op") == "publish" and
                                  m["msg"]["transforms"][0]["header"]["stamp"]["sec"] == 8)
            self.assertAlmostEqual(reset_tf["msg"]["transforms"][0]
                                   ["transform"]["rotation"]["z"], 0.0)

            pose = External(connect())
            pose.send({"op": "advertise", "topic": "/global_pose", "type": "geometry_msgs/Pose"})
            pose.send({"op": "publish", "topic": "/global_pose",
                       "msg": {"position": {"x": 5, "y": 0, "z": 0},
                               "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}})
            moved_world = xform.receive(lambda m: m.get("op") == "publish" and
                                        any(e["child_frame_id"] == "base" and e["matrix"][12] == 5
                                            for e in m["msg"]["transforms"]))
            moved = {e["child_frame_id"]: e for e in moved_world["msg"]["transforms"]}
            self.assertAlmostEqual(moved["tip"]["matrix"][12], 6.0)
            pose.close()

            start = time.monotonic()
            for _ in range(5):
                tf.receive(lambda m: m.get("op") == "publish" and
                           m["msg"]["transforms"][0]["header"]["stamp"]["sec"] == 8)
            self.assertLess(time.monotonic() - start, 1.2)

            invalid = service.call("/param_server/set_param",
                                   {"name": "robot_description", "value": "<robot/>"})
            rejected = self.wait_status(service, invalid["values"]["version"])
            self.assertFalse(rejected["accepted"])
            self.assertEqual(rejected["loaded_version"], version)

            newer = service.call("/param_server/set_param",
                                 {"name": "robot_description", "value": OTHER})
            accepted = self.wait_status(service, newer["values"]["version"])
            self.assertTrue(accepted["accepted"])
            latest_tf = tf.receive(lambda m: m.get("op") == "publish" and
                                   m["msg"]["transforms"][0]["child_frame_id"] == "new_tip")
            self.assertEqual(latest_tf["msg"]["transforms"][0]["header"]["frame_id"], "root")
            latest_world = xform.receive(lambda m: m.get("op") == "publish" and
                                         {e["child_frame_id"] for e in m["msg"]["transforms"]}
                                         == {"root", "new_tip"})
            self.assertEqual(len(latest_world["msg"]["transforms"]), 2)
            joint.close()
        finally:
            robot.running = False
            world.running = False
            for client in (param_client, robot_client, world_client):
                client.close()
            for external in (service, tf, xform):
                external.close()
            gateway.running = False

    @staticmethod
    def wait_status(service, version):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            reply = service.call("/param_server/get_param",
                                 {"name": "/robot_state_publisher/description_status"})
            value = reply["values"]["value"]
            if value and value["version"] >= version:
                return value
            time.sleep(0.05)
        raise AssertionError("description status missed two-second deadline")


if __name__ == "__main__":
    unittest.main()
