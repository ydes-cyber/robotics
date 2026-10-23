"""Generic parameter server; URDF validity belongs to robot_state_publisher."""
import threading


class ParameterServer:
    def __init__(self, client):
        self.values = {}
        self.lock = threading.Lock()
        client.advertise_service("/param_server/set_param", "param_server/SetParam", self.set_param)
        client.advertise_service("/param_server/get_param", "param_server/GetParam", self.get_param)

    def set_param(self, args):
        name = args.get("name") if isinstance(args, dict) else None
        if not isinstance(name, str):
            return {}, False, "name must be a string"
        value = args.get("value")
        with self.lock:
            version = self.values[name][1] + 1 if name in self.values else 1
            self.values[name] = (value, version)
        return {"version": version}, True, ""

    def get_param(self, args):
        name = args.get("name") if isinstance(args, dict) else None
        if not isinstance(name, str):
            return {"value": None, "version": 0}, False, "name must be a string"
        with self.lock:
            entry = self.values.get(name)
        if entry is None:
            return {"value": None, "version": 0}, False, "parameter not set"
        return {"value": entry[0], "version": entry[1]}, True, ""
