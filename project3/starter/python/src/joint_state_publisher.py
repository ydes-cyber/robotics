"""Recommended demo joint driver; never started by make run."""
import threading
import time

from urdf_parser import parse_robot_description


class JointStatePublisher:
    def __init__(self, client):
        self.client = client
        self.lock = threading.Lock()
        self.robot = None
        self.version = 0
        self.positions = {}
        self.targets = {}
        client.subscribe("/joint_trajectory", "trajectory_msgs/JointTrajectory", self.on_trajectory)
        client.advertise("/joint_states", "sensor_msgs/JointState")
        client.advertise_service("/joint_state_publisher/reset", "std_srvs/Trigger", self.reset)
        self.running = True
        threading.Thread(target=self.run, daemon=True).start()

    def on_trajectory(self, message):
        points = message.get("points", []) if isinstance(message, dict) else []
        if not points:
            return
        names = message.get("joint_names", [])
        values = points[-1].get("positions", [])
        with self.lock:
            self.targets.update(zip(names, values))

    def reset(self, _args):
        with self.lock:
            if self.robot is None:
                return {}, False, "no robot loaded"
            for joint in self.robot.joints:
                if joint.kind == "fixed":
                    continue
                value = (joint.lower + joint.upper) / 2 if joint.kind != "continuous" else 0.0
                self.positions[joint.name] = value
                self.targets[joint.name] = value
        return {}, True, ""

    def poll_description(self):
        reply = self.client.call("/param_server/get_param", {"name": "robot_description"})
        if not reply or not reply.get("result"):
            return
        version = reply["values"]["version"]
        if version == self.version:
            return
        self.version = version
        try:
            robot = parse_robot_description(reply["values"]["value"])
        except (ValueError, TypeError):
            return
        with self.lock:
            self.robot = robot
            names = {joint.name for joint in robot.joints if joint.kind != "fixed"}
            self.positions = {name: self.positions.get(name, 0.0) for name in names}

    def publish_state(self):
        with self.lock:
            if self.robot is None:
                return
            names = []
            for joint in self.robot.joints:
                if joint.kind == "fixed":
                    continue
                name = joint.name
                target = self.targets.get(name, self.positions.get(name, 0.0))
                if joint.kind != "continuous":
                    target = max(joint.lower, min(joint.upper, target))
                self.positions[name] = target  # snapping is allowed for the demo node
                names.append(name)
            values = [self.positions[name] for name in names]
        now = time.time_ns()
        stamp = {"sec": now // 1_000_000_000, "nanosec": now % 1_000_000_000}
        self.client.publish("/joint_states", {"header": {"stamp": stamp, "frame_id": ""},
                                              "name": names, "position": values,
                                              "velocity": [], "effort": []})

    def run(self):
        next_poll = 0.0
        while self.running:
            if time.monotonic() >= next_poll:
                try:
                    self.poll_description()
                except (OSError, TimeoutError, KeyError) as exc:
                    print(f"demo description poll error: {exc}", flush=True)
                next_poll = time.monotonic() + 0.3
            try:
                self.publish_state()
            except (OSError, ValueError, TypeError) as exc:
                print(f"demo joint publication error: {exc}", flush=True)
            time.sleep(0.1)
