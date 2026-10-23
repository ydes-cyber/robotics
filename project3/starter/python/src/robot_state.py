"""robot_state_publisher: description lifecycle, joint states, and /tf."""
import threading
import time

from forward_kinematics import tf_message
from urdf_parser import parse_robot_description

ZERO_STAMP = {"sec": 0, "nanosec": 0}


class RobotStatePublisher:
    def __init__(self, client):
        self.client = client
        self.lock = threading.RLock()
        self.robot = None
        self.seen_version = 0
        self.loaded_version = 0
        self.positions = {}
        self.stamp = ZERO_STAMP.copy()
        client.subscribe("/joint_states", "sensor_msgs/JointState", self.on_joint_states)
        client.advertise("/tf", "tf/tfMessage")
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def on_joint_states(self, message):
        if not isinstance(message, dict):
            return
        names = message.get("name", [])
        values = message.get("position", [])
        with self.lock:
            if self.robot is None:
                return
            movable = {joint.name for joint in self.robot.joints if joint.kind != "fixed"}
            self.positions = {name: value for name, value in zip(names, values)
                              if name in movable}
            self.stamp = message["header"]["stamp"]

    def poll_description(self):
        reply = self.client.call("/param_server/get_param", {"name": "robot_description"})
        if not reply or not reply.get("result"):
            return
        values = reply["values"]
        version = values["version"]
        if version == self.seen_version:
            return
        try:
            candidate = parse_robot_description(values["value"])
        except (ValueError, TypeError, OverflowError) as exc:
            accepted, error = False, str(exc)
        else:
            with self.lock:
                self.robot = candidate
                self.loaded_version = version
                self.positions = {}
                self.stamp = ZERO_STAMP.copy()
            accepted, error = True, ""
        with self.lock:
            status = {"version": version, "accepted": accepted, "error": error,
                      "loaded_version": self.loaded_version,
                      "root_link": self.robot.root if self.robot else ""}
        # The swap waits for any in-flight /tf send under self.lock. Do not hold
        # that lock across a service call: the client reader handles joint states.
        response = self.client.call("/param_server/set_param",
                                    {"name": "/robot_state_publisher/description_status",
                                     "value": status})
        if response and response.get("result"):
            self.seen_version = version

    def publish_tf(self):
        with self.lock:
            if self.robot is not None:
                self.client.publish("/tf", tf_message(self.robot, self.positions, self.stamp))

    def run(self):
        next_poll = 0.0
        while self.running:
            now = time.monotonic()
            if now >= next_poll:
                try:
                    self.poll_description()
                except (OSError, TimeoutError, KeyError) as exc:
                    print(f"description poll error: {exc}", flush=True)
                next_poll = now + 0.1
            try:
                self.publish_tf()
            except (OSError, ValueError) as exc:
                print(f"tf publication error: {exc}", flush=True)
            time.sleep(0.1)
