"""robot_world_state_publisher: compose latest /tf tree with /global_pose."""
import threading
import time

from forward_kinematics import matrix_message
from transforms import compose, identity, quaternion_to_matrix, translation


def edge_matrix(entry):
    transform = entry["transform"]
    position = transform["translation"]
    rotation = transform["rotation"]
    xyz = (position["x"], position["y"], position["z"])
    quat = (rotation["x"], rotation["y"], rotation["z"], rotation["w"])
    return compose(translation(xyz), quaternion_to_matrix(quat))


class RobotWorldStatePublisher:
    def __init__(self, client):
        self.client = client
        self.lock = threading.Lock()
        self.latest_tf = None
        self.global_root = identity()
        client.subscribe("/tf", "tf/tfMessage", self.on_tf)
        client.subscribe("/global_pose", "geometry_msgs/Pose", self.on_global_pose)
        client.advertise("/xform_world", "xform_world/MatrixTransformArray")
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.empty_root = ""
        self.last_status_poll = 0.0
        self.thread.start()
        
    def refresh_description_status(self):
        if time.monotonic() - self.last_status_poll < 0.5:
            return

        self.last_status_poll = time.monotonic()
        reply = self.client.call(
            "/param_server/get_param",
            {"name": "/robot_state_publisher/description_status"},
        )
        if not reply or not reply.get("result"):
            return
        status = reply.get("values", {}).get("value")
        if not isinstance(status, dict) or not status.get("accepted"):
            return
        root = status.get("root_link")
        if isinstance(root, str) and root:
            with self.lock:
                self.empty_root = root
                
    def on_tf(self, message):
        if isinstance(message, dict):
            with self.lock:
                self.latest_tf = message

    def on_global_pose(self, message):
        position = message["position"]
        orientation = message["orientation"]
        xyz = (position["x"], position["y"], position["z"])
        quat = (orientation["x"], orientation["y"], orientation["z"], orientation["w"])
        with self.lock:
            self.global_root = compose(translation(xyz), quaternion_to_matrix(quat))

    def publish_world(self):
        with self.lock:
            tf, root_pose = self.latest_tf, self.global_root
          
        if tf is None:
            return
          
        edges = tf.get("transforms", [])
        # robot with one link and no joints.
        if not edges:
            with self.lock:
                root = self.empty_root

            if not root:
                return
              
            self.client.publish("/xform_world",matrix_message({root: root_pose},
                               {"sec": 0, "nanosec": 0},),)
            return
        children = {entry["child_frame_id"] for entry in edges}
        parents = {entry["header"]["frame_id"] for entry in edges}
        roots = parents - children

        if len(roots) != 1:
            return

        root = roots.pop()
        adjacency = {}

        for entry in edges:
            adjacency.setdefault(entry["header"]["frame_id"], []).append(entry)

        transforms = {root: root_pose}
        stack = [root]

        while stack:
            parent = stack.pop()

            for entry in adjacency.get(parent, []):
                child = entry["child_frame_id"]
                transforms[child] = compose(
                    transforms[parent],
                    edge_matrix(entry),)
                stack.append(child)

        stamp = edges[0]["header"]["stamp"]
        self.client.publish("/xform_world",matrix_message(transforms, stamp),)
        
    def run(self):
        while self.running:
            try:
                self.refresh_description_status()
                self.publish_world()
            except (OSError, TimeoutError, ValueError, KeyError, TypeError) as exc:
                print(f"world publication error: {exc}", flush=True)
            time.sleep(0.1)