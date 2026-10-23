"""Recommended joint-space choreography node for make demo."""
import json
import re
import threading
import time
from pathlib import Path

FILENAME = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")
FSM_DIR = Path(__file__).resolve().parents[1] / "fsm"


def validate_logic(document):
    if not isinstance(document, dict):
        raise ValueError("FSM document must be an object")
    states = document.get("states")
    if not isinstance(states, list) or not states:
        raise ValueError("FSM needs states")
    by_name = {}
    for state in states:
        name = state.get("name")
        if not isinstance(name, str) or name in by_name:
            raise ValueError("invalid or duplicate state name")
        target = state.get("target", {})
        names, values = target.get("joint_names"), target.get("positions")
        if not isinstance(names, list) or not isinstance(values, list) or len(names) != len(values):
            raise ValueError("target names and positions differ")
        epsilon = state.get("epsilon", 0.02)
        if not isinstance(epsilon, (int, float)) or epsilon < 0:
            raise ValueError("invalid epsilon")
        by_name[name] = state
    if document.get("initial_state") not in by_name:
        raise ValueError("initial state does not exist")
    for state in states:
        transitions = state.get("transitions", [])
        names = [item.get("name") for item in transitions]
        if len(names) != len(set(names)) or any(item.get("next_state") not in by_name for item in transitions):
            raise ValueError("invalid transition")
        if state.get("on_arrival") is not None and state["on_arrival"] not in names:
            raise ValueError("on_arrival does not name a transition")
    return by_name


class FiniteStateMachine:
    def __init__(self, client):
        self.client = client
        self.lock = threading.RLock()
        self.logic = None
        self.by_name = {}
        self.current = None
        self.entered = time.monotonic()
        self.last_transition = None
        self.running = False
        self.arrived = False
        client.subscribe("/joint_states", "sensor_msgs/JointState", self.on_joint_states)
        client.advertise("/joint_trajectory", "trajectory_msgs/JointTrajectory")
        client.advertise("/fsm/status", "fsm/Status")
        handlers = {
            "update_logic": self.update_logic, "get_logic": self.get_logic,
            "set_running": self.set_running, "fire_transition": self.fire_transition,
            "save_to_file": self.save_to_file, "load_from_file": self.load_from_file,
            "list_saved": self.list_saved,
        }
        for name, handler in handlers.items():
            client.advertise_service(f"/fsm/{name}", "fsm/Service", handler)
        self.active = True
        threading.Thread(target=self.run, daemon=True).start()

    def enter_state(self, name, transition=None):
        self.current = name
        self.entered = time.monotonic()
        self.last_transition = transition
        self.arrived = False
        target = self.by_name[name]["target"]
        self.client.publish("/joint_trajectory", {
            "joint_names": target["joint_names"],
            "points": [{"positions": target["positions"], "velocities": [],
                        "accelerations": [], "time_from_start": {"sec": 0, "nanosec": 0}}],
        })

    def update_logic(self, args):
        by_name = validate_logic(args)
        with self.lock:
            self.logic = args
            self.by_name = by_name
            self.running = True
            self.enter_state(args["initial_state"])
            return {"current_state": self.current}, True, ""

    def get_logic(self, _args):
        with self.lock:
            if self.logic is None:
                return {}, False, "no logic loaded"
            return dict(self.logic), True, ""

    def set_running(self, args):
        with self.lock:
            if isinstance(args, dict) and "data" in args:
                if not isinstance(args["data"], bool):
                    return {"running": self.running}, False, "data must be boolean"
                self.running = args["data"]
            return {"running": self.running}, True, ""

    def take_transition(self, name):
        state = self.by_name[self.current]
        match = next((item for item in state.get("transitions", []) if item.get("name") == name), None)
        if match is None:
            return False
        self.enter_state(match["next_state"], name)
        return True

    def fire_transition(self, args):
        with self.lock:
            if not self.running or self.current is None:
                return {}, False, "FSM is stopped"
            if not self.take_transition(args.get("transition") if isinstance(args, dict) else None):
                return {}, False, "unknown transition"
            return {"current_state": self.current}, True, ""

    def on_joint_states(self, message):
        with self.lock:
            if not self.running or self.current is None or self.arrived:
                return
            state = self.by_name[self.current]
            target = state["target"]
            positions = dict(zip(message.get("name", []), message.get("position", [])))
            epsilon = state.get("epsilon", 0.02)
            if all(name in positions and abs(positions[name] - value) <= epsilon
                   for name, value in zip(target["joint_names"], target["positions"])):
                self.arrived = True
                if state.get("on_arrival"):
                    self.take_transition(state["on_arrival"])

    @staticmethod
    def path(args):
        identifier = args.get("identifier") if isinstance(args, dict) else None
        if not isinstance(identifier, str) or not FILENAME.fullmatch(identifier):
            raise ValueError("invalid identifier")
        return FSM_DIR / f"{identifier}.json"

    def save_to_file(self, args):
        path = self.path(args)
        FSM_DIR.mkdir(exist_ok=True)
        document = {"initial_state": args.get("initial_state"), "states": args.get("states")}
        path.write_text(json.dumps(document), encoding="utf-8")
        return {"path": str(path)}, True, ""

    def load_from_file(self, args):
        path = self.path(args)
        try:
            return json.loads(path.read_text(encoding="utf-8")), True, ""
        except FileNotFoundError:
            return {}, False, "file not found"

    def list_saved(self, _args):
        names = sorted(path.stem for path in FSM_DIR.glob("*.json") if FILENAME.fullmatch(path.stem))
        return {"identifiers": names}, True, ""

    def run(self):
        while self.active:
            with self.lock:
                status = {"current_state": self.current,
                          "time_in_state": time.monotonic() - self.entered,
                          "last_transition": self.last_transition, "running": self.running}
            try:
                self.client.publish("/fsm/status", status)
            except OSError:
                break
            time.sleep(0.1)
