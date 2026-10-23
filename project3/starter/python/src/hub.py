"""Project 1 topic and service routing, adapted for the Project 3 wire contract."""
import itertools
import json
import queue
import socket
import threading
import time 


class Peer:
    def __init__(self, sock, on_failure=None):
        self.sock = sock
        self.send_lock = threading.Lock()
        self.alive = True
        self.outgoing = queue.Queue(maxsize=32)
        self.on_failure = on_failure
        threading.Thread(target=self._write, daemon=True).start()

    def send(self, message):
        payload = (json.dumps(message, ensure_ascii=True, allow_nan=False,
                              separators=(",", ":")) + "\n").encode("utf-8")
        if not self.alive:
            return False
        try:
            self.outgoing.put_nowait(payload)
            return True
        except queue.Full:
            return False

    def _write(self):
        while self.alive:
            try:
                payload = self.outgoing.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                with self.send_lock:
                    self.sock.sendall(payload)
            except OSError:
                if self.on_failure:
                    self.on_failure(self)
                else:
                    self.close()
                break

    def close(self):
        if not self.alive:
            return
        self.alive = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


class Hub:
    def __init__(self):
        self.lock = threading.RLock()
        self.publishers = {}
        self.subscribers = {}
        self.services = {}
        self.pending = {}
        self.ids = itertools.count()
        self.timeout_thread = threading.Thread(target=self._timeout_loop, daemon=True)
        self.timeout_thread.start()

    @staticmethod
    def status(message, level, detail):
        reply = {"op": "status", "level": level, "msg": detail}
        if "id" in message:
            reply["id"] = message["id"]
        return reply

    @staticmethod
    def failed(service, call_id, detail):
        return {"op": "service_response", "service": service, "id": call_id,
                "values": {}, "result": False, "status": detail}

    def _expire(self, provider_id):
        with self.lock:
            call = self.pending.pop(provider_id, None)
        if call is not None:
            call[0].send(self.failed(call[3], call[1], "service provider timeout"))


    def _timeout_loop(self):
        while True:
            time.sleep(0.1)
            with self.lock:
                expired = [
                    (provider_id, call)
                    for provider_id, call in self.pending.items()
                    if call[4] <= time.monotonic()
                ]
                for provider_id, _ in expired:
                    self.pending.pop(provider_id, None)

            for _, call in expired:
                caller, call_id, _, service, _ = call
                caller.send(
                    self.failed(
                        service,
                        call_id,
                        "service provider timeout",
                    )
                )

    def dispatch(self, peer, message):
        if not isinstance(message, dict):
            peer.send(self.status({}, "error", "expected JSON object"))
            return
        op = message.get("op")
        if op in ("advertise", "unadvertise", "subscribe", "unsubscribe", "publish"):
            self._topic(peer, message, op)
        elif op in ("advertise_service", "unadvertise_service", "call_service", "service_response"):
            self._service(peer, message, op)
        else:
            peer.send(self.status(message, "error", "unknown or missing op"))

    def _topic(self, peer, message, op):
        topic = message.get("topic")
        if not isinstance(topic, str) or not topic.startswith("/"):
            peer.send(self.status(message, "error", "invalid topic"))
            return
        if op in ("advertise", "subscribe") and not isinstance(message.get("type"), str):
            peer.send(self.status(message, "error", "missing type"))
            return
        if op == "publish":
            if "msg" not in message:
                peer.send(self.status(message, "error", "missing msg"))
                return
            with self.lock:
                recipients = tuple(self.subscribers.get(topic, ()))
            packet = {"op": "publish", "topic": topic, "msg": message["msg"]}
            for recipient in recipients:
                if not recipient.send(packet):
                    self.disconnect(recipient)
            return
        with self.lock:
            if op == "advertise":
                self.publishers.setdefault(topic, set()).add(peer)
            elif op == "unadvertise":
                self.publishers.get(topic, set()).discard(peer)
            elif op == "subscribe":
                self.subscribers.setdefault(topic, set()).add(peer)
            else:
                self.subscribers.get(topic, set()).discard(peer)
        if op == "subscribe":
            peer.send(self.status(message, "info", f"subscribed to {topic}"))

    def _service(self, peer, message, op):
        service = message.get("service")
        if not isinstance(service, str) or not service.startswith("/"):
            peer.send(self.status(message, "error", "invalid service"))
            return
        if op == "advertise_service":
            if not isinstance(message.get("type"), str):
                peer.send(self.status(message, "error", "missing type"))
                return
            with self.lock:
                self.services[service] = peer
            peer.send(self.status(message, "info", f"advertised {service}"))
        elif op == "unadvertise_service":
            with self.lock:
                if self.services.get(service) is peer:
                    del self.services[service]
        elif op == "call_service":
            if "id" not in message or "args" not in message:
                peer.send(self.status(message, "error", "missing call id or args"))
                return
            with self.lock:
                provider = self.services.get(service)
                if provider is not None:
                    provider_id = f"provider-call-{next(self.ids)}"
                    self.pending[provider_id] = (
                        peer,
                        message["id"],
                        provider,
                        service,
                        time.monotonic() + 3.0,
                    )
            if provider is None:
                peer.send(self.failed(service, message["id"], "missing service provider"))
            elif not provider.send({"op": "call_service", "service": service,
                                    "id": provider_id, "args": message["args"]}):
                self.disconnect(provider)
        else:
            provider_id = message.get("id")
            with self.lock:
                call = self.pending.get(provider_id) if isinstance(provider_id, str) else None
                if call and call[2] is peer and call[3] == service:
                    del self.pending[provider_id]
                else:
                    call = None
            if call is None:
                peer.send(self.status(message, "error", "unknown service response"))
            elif "values" not in message or "result" not in message:
                call[0].send(self.failed(service, call[1], "malformed provider response"))
            else:
                call[0].send({"op": "service_response", "service": service,
                              "id": call[1], "values": message["values"],
                              "result": message["result"], "status": message.get("status", "")})

    def disconnect(self, peer):
        failed = []
        with self.lock:
            for owners in self.publishers.values():
                owners.discard(peer)
            for readers in self.subscribers.values():
                readers.discard(peer)
            for service, provider in tuple(self.services.items()):
                if provider is peer:
                    del self.services[service]
            for provider_id, call in tuple(self.pending.items()):
                if call[2] is peer:
                    failed.append(call)
                    del self.pending[provider_id]
                elif call[0] is peer:
                    del self.pending[provider_id]
        peer.close()
        for caller, call_id, _, service, _ in failed:
            caller.send(self.failed(service, call_id, "provider disconnected"))