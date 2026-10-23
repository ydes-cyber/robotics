"""Newline-delimited TCP gateway for the Project 1-compatible hub."""
import json
import socket
import threading

from hub import Hub, Peer

HOST = "127.0.0.1"
PORT = 9095
MAX_LINE = 4 * 1024 * 1024


def reject_constant(value):
    raise ValueError(f"invalid JSON constant: {value}")


class Gateway:
    def __init__(self):
        self.hub = Hub()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST, PORT))
        self.sock.listen()
        self.running = True
        self.thread = threading.Thread(target=self._accept, daemon=True)

    def start(self):
        self.thread.start()

    def _accept(self):
        while self.running:
            try:
                sock, _ = self.sock.accept()
            except OSError:
                break
            peer = Peer(sock, self.hub.disconnect)
            threading.Thread(target=self._client, args=(peer,), daemon=True).start()

    def _client(self, peer):
        try:
            # makefile handles partial TCP frames and split UTF-8 characters.
            with peer.sock.makefile("rb") as stream:
                while self.running:
                    line = stream.readline(MAX_LINE + 2)
                    if not line:
                        break
                    if len(line) > MAX_LINE + 1 or not line.endswith(b"\n"):
                        peer.send({"op": "status", "level": "error", "msg": "line too long"})
                        break
                    if not line.strip():
                        continue
                    try:
                        message = json.loads(line.decode("utf-8"), parse_constant=reject_constant)
                    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                        peer.send({"op": "status", "level": "error", "msg": "invalid JSON"})
                        continue
                    self.hub.dispatch(peer, message)
        except OSError:
            pass
        finally:
            self.hub.disconnect(peer)

    def close(self):
        self.running = False
        self.sock.close()
