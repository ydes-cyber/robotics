
import socket # networking tools
import json
HOST = "127.0.0.1" #localhost (listen on my computer only)
PORT = 9095 # use port the assignment says to use
#this is creating a scoket use normal internet address,and use TCP
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# attach server to address and port 
server.bind((HOST, PORT))
server.listen()

print(f"Listening on {HOST}:{PORT}", flush = True)
#flush means put output on screen immmediately 
topics = {}
subs = {}
while True:
    connection, address = server.accept() # basically means wait until another program connects to me and .accept() takes two pieces of info address and connection
    print(f"Connected: {address}", flush=True)
    read_data = connection.recv(4096) # 4096 is the number max number of bytes were accpeting as a message
    message = read_data.decode()
    request = json.loads(message) ## turns into python dict 
    if request["op"] == "advertise":
        topics[request["topic"]] = request["type"]
        print(f"Advertised {request['topic']}", flush=True)
    elif request["op"] == "publish":
        print("Publish request", flush=True)
    elif request["op"] == "unadvertise":
        print("Unadvertise request", flush=True)
    elif request["op"] == "subscribe":
        topic = request["topic"]
        subs.setdefault(topic, []).append(connection) # this finds list of clients subscribed to /map if it doesn't exist yet, create one then add client
        print("Subscribed to {topic}", flush=True)

    elif request["op"] == "unsubscribe":
        print("Unsubscribe request", flush=True)

    elif request["op"] == "call_service":
        print("Service call request", flush=True)
    connection.close() # close because were finished with this connection 
  ## Note to self: setdefault returns value of a key if  it exist if it doesn't it inserts key with a specific default value and returns it