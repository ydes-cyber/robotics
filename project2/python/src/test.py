import socket
import json
#test.py
# 1. Connect to the local server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(("127.0.0.1", 9095))

# 2. Build a standard rosbridge JSON service call
test_message = {
    "op": "call_service",
    "service": "/ik/solve",
    "id": 2,
    "args": {
        "x": 1.0,
        "y": 0.0
    }
}

# 3. Package as a string with a newline and send it
client.sendall((json.dumps(test_message) + "\n").encode())

# 4. Wait for the server's reply and print it
response = client.recv(1024).decode()
print("Server replied:", response)

client.close()