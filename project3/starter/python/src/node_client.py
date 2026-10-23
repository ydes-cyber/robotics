"""Small Project 3 node client over the same rosbridge TCP protocol."""
import itertools
import json
import socket
import threading

from rosbridge import HOST, PORT, MAX_LINE


class NodeClient:
    def __init__(self):
        self.sock = socket.create_connection((HOST, PORT))
        self.send_lock = threading.Lock()
        self.lock = threading.Lock()
        self.pending = {}
        self.services = {}
        self.topics = {}
        self.ids = itertools.count()
        self.running = True
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def send(self, message):
        payload = (json.dumps(message, ensure_ascii=True, allow_nan=False,
                              separators=(",", ":")) + "\n").encode("utf-8")
        with self.send_lock:
            self.sock.sendall(payload)

    def advertise(self, topic, type_name):
        self.send({"op": "advertise", "topic": topic, "type": type_name})

    def publish(self, topic, message):
        self.send({"op": "publish", "topic": topic, "msg": message})

    def subscribe(self, topic, type_name, callback):
        self.topics[topic] = callback
        self.send({"op": "subscribe", "topic": topic, "type": type_name})

    def advertise_service(self, service, type_name, callback):
        self.services[service] = callback
        self.send({"op": "advertise_service", "service": service, "type": type_name})

    def call(self, service, args, timeout=4.0):
        call_id = f"node-{id(self)}-{next(self.ids)}"
        ready = threading.Event()
        slot = [None, ready]
        with self.lock:
            self.pending[call_id] = slot
        try:
            self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
            if not ready.wait(timeout):
                raise TimeoutError(f"service call timed out: {service}")
            return slot[0]
        finally:
            with self.lock:
                self.pending.pop(call_id, None)

    def _read(self):
        try:
            with self.sock.makefile("rb") as stream:
                while self.running:
                    line = stream.readline(MAX_LINE + 2)
                    if not line:
                        break
                    if len(line) > MAX_LINE + 1 or not line.endswith(b"\n"):
                        break
                    try:
                        message = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    op = message.get("op") if isinstance(message, dict) else None
                    if op == "service_response":
                        with self.lock:
                            slot = self.pending.get(message.get("id"))
                        if slot:
                            slot[0] = message
                            slot[1].set()
                    elif op == "publish":
                        callback = self.topics.get(message.get("topic"))
                        if callback:
                            try:
                                callback(message.get("msg"))
                            except Exception as exc:
                                print(f"topic callback error: {exc}", flush=True)
                    elif op == "call_service":
                        callback = self.services.get(message.get("service"))
                        if callback:
                            try:
                                values, result, status = callback(message.get("args"))
                            except Exception as exc:
                                values, result, status = {}, False, str(exc)
                            self.send({"op": "service_response", "service": message["service"],
                                       "id": message["id"], "values": values,
                                       "result": result, "status": status})
        except OSError:
            pass
        finally:
            self.running = False
            with self.lock:
                for slot in self.pending.values():
                    slot[1].set()

    def close(self):
        self.running = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
