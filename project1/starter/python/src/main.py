"""Project 1 starter: replace this foreground placeholder with your runtime."""
from email.mime import message
import os 
import sys
import socket 
import json
import threading
from urllib import request 
host = "127.0.0.1"
port = 9095
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from autorob.heap_sort_node import heap_sort, heapify
from autorob.astar_node import (get_map_data,astar,alter_to_map_coord,cell_to_world_coord, map_is_valid)

def send_josn(connection, message):
    message = json.dumps(message)
    connection.sendall(message.encode())
    
def time_service(provider_id):
    with service_lock:
        call_info = forwarded_calls.pop(provider_id, None)
        if call_info is None:
            return 
        response  = {
            "op": "service_response",
            "service": call_info["service"],
            "id": call_info["caller_id"],
            "values": {},
            "result": False,
            "status": "service provider timeout"
        }
        try:
            call_info["caller"].sendall((json.dumps(response) + "\n").encode())
        except OSError:
            pass
        

    
    
    
    
def handle_request(connection, request):
## This part insures that the server can handle a call_service request 
# for the /heap_sort service. It extracts the numbers from the request, sorts them using the heap_sort function, and prints the sorted list to the console.
    global next_provider_call_id
    if request["op"] == "autorob_readiness_probe":
      response = {
          "op": "status",
          "level": "error",
          "msg": "unknown operation",
          "id": request["id"]
      }
      connection.sendall((json.dumps(response) + "\n").encode())
      return
    if request["op"] == "advertise_service":
        service = request["service"]
        with service_lock:
            employees[service] = connection
            
        response = {
            "op": "status",
            "level": "info",
            "msg": f"advertised service {service}",
            "id": request.get("id")
        }
        connection.sendall((json.dumps(response) + "\n").encode())
        return 
    if request["op"] == "call_service":
        service = request["service"]
        if service == "/heap_sort":
            numbers = request["args"]["numbers"] # extract the numbers from the request, args is a dictionary with key numbers and value is the list of numbers to be sorted
            sorted_list = heap_sort(numbers)
            print(sorted_list, flush=True)
            response_dict ={
                "op": "service_response",
                "service": "/heap_sort", # this is the name of the service we are responding to
                "id": request["id"], # this is the id of the request we are responding to
                "values": {"sorted": sorted_list}, # this is the sorted list we are returning as the result of the service call
                "result": True, # resuts is true because the service call was successful
                "status": "" # status is an empty string because we don't have any errors to report
            }
        elif service == "/heapify":
            values = request["args"]["values"]
            heap_results = heapify(values)
            response_dict = {
                "op": "service_response",
                "service": "/heapify",
                "id": request["id"],
                "values": {"heap": heap_results},
                "result": True,
                "status": ""
            }
        elif service =="/plan_path":
            try:
                start = request["args"]["start"]["pose"]["position"]
                target = request["args"]["goal"]["pose"]["position"]
                start_point = alter_to_map_coord(start["x"],start["y"])
                end_point = alter_to_map_coord(target["x"],target["y"])
                spots = astar(start_point, end_point)
                
                if spots is None: 
                    response_dict = {
                        "op": "service_response",
                        "service": "/plan_path",
                        "id": request["id"],
                        "values": {},
                        "result": False,
                        "status": "no path found"
                    }
                else:
                    poses = []
                    
                    for x, y in spots:
                        world_x, world_y = cell_to_world_coord(x,y)
                        poses.append({
                            "header": {"frame_id": "map"},
                            "pose": {
                                    "position": {
                                    "x": world_x,
                                    "y": world_y,
                                    "z": 0.0},
                                    "orientation": {
                                        "x": 0.0,
                                        "y": 0.0,
                                        "z": 0.0,
                                        "w": 1.0 }
                                }
                        })
                    plan = {
                        "header": {"frame_id": "map"},
                        "poses": poses }

                    response_dict = {
                        "op": "service_response",
                        "service": "/plan_path",
                        "id": request["id"],
                        "values": {"plan": plan},
                        "result": True,
                        "status": ""
                    }
                    with topic_lock:
                        path_listeners = list(subscribers.get("/path", []))
                    for listener in path_listeners:
                        publication = {
                              "op": "publish",
                              "topic": "/path",
                              "msg": plan
                        }

                        try:
                            listener.sendall((json.dumps(publication) +"\n").encode())
                        except OSError:
                            with topic_lock:
                                  if listener in subscribers.get("/path", []):
                                    subscribers["/path"].remove(listener)
            except (KeyError, TypeError, ValueError):
                    response_dict = {
                      "op": "service_response",
                      "service": "/plan_path",
                      "id": request["id"],
                      "values": {},
                      "result": False,
                      "status": "invalid plan request"
                  }
        else:
            with service_lock:
                provider = employees.get(service)
                if provider is None:
                    response_dict = {
                        "op": "service_response",
                        "service": service,
                        "id": request["id"],
                        "values": {},
                        "result": False,
                        "status": "missing service provider"
                        }
                else:
                    provider_id = f"provider-call-{next_provider_call_id}"
                    next_provider_call_id += 1
                    forwarded_calls[provider_id] = {
                          "caller": connection,
                          "caller_id": request["id"],
                          "service": service,
                          "provider": provider 
                    }
                    timer = threading.Timer(
                        3.0,
                        time_service,
                        args=(provider_id,) 
                    )
                    timer.daemon = True
                    timer.start()
            if provider is None:
                connection.sendall((json.dumps(response_dict) +
                "\n").encode())
                return

            forward_request = {
                  "op": "call_service",
                  "service": service,
                  "id": provider_id,
                  "args": request["args"]
            }
            provider.sendall((json.dumps(forward_request) + "\n").encode())
            return

        connection.sendall((json.dumps(response_dict) + "\n").encode())
    
    elif request["op"] == "service_response":
        provider_id = request["id"]

        with service_lock:
            original_call = forwarded_calls.pop(provider_id, None)

        if original_call is not None:
            response = {
                  "op": "service_response",
                  "service": original_call["service"],
                  "id": original_call["caller_id"],
                  "values": request.get("values", {}),
                  "result": request.get("result", False),
                  "status": request.get("status", "")
            }

            caller = original_call["caller"]
            caller.sendall((json.dumps(response) + "\n").encode())
    elif request["op"] == "advertise":
        topic =request["topic"]
        with topic_lock:
            publishers.setdefault(topic, []).append(connection)
            
    elif request["op"] =="unadvertise": 
        topic = request["topic"]
        with topic_lock:
            if topic in publishers and connection in publishers[topic]:
                publishers[topic].remove(connection)    
    elif request["op"] == "subscribe":
        topic = request["topic"]
        
        with topic_lock:
            subscribers.setdefault(topic, []).append(connection)
            response = {
              "op": "status",
              "level": "info",
              "msg": f"subscribed to {topic}",
              "id": request.get("id")
          }
            connection.sendall((json.dumps(response) + "\n").encode())

    elif request["op"] == "unsubscribe":
        topic = request["topic"]

        with topic_lock:
            if topic in subscribers and connection in subscribers[topic]:
                subscribers[topic].remove(connection)
                  
    elif request["op"] == "publish":
        topic = request["topic"]
        payload = request["msg"]
        with topic_lock: 
            if connection not in publishers.get(topic, []):
                return
        if topic == "/map" and map_is_valid(payload):
                get_map_data(payload)
                print("Map data received", flush=True)

        with topic_lock:
            listeners = list(subscribers.get(topic, []))

        for listener in listeners:
            publication = {
                "op": "publish",
                "topic": topic,
                "msg": payload
              }
            try:
                listener.sendall((json.dumps(publication) + "\n").encode())
            except OSError:
                with topic_lock:
                    if listener in subscribers.get(topic, []):
                        subscribers[topic].remove(listener)
    elif request["op"] == "unadvertise_service":
        service = request["service"]
        with service_lock: 
            if employees.get(service) == connection:
                del employees[service]
        response = {
            "op": "status",
            "level": "info",
            "msg": f"unadvertised service {service}",
            "id": request.get("id")
        }
        connection.sendall((json.dumps(response) + "\n").encode())
def handle_client(connection, address):
    print(f"Connected: {address}", flush=True) # this prints the address of the client that just connected to the server
    buffer = "" # This buffer will hold any partial messages received from the client. It allows us to handle cases where a message is split across multiple recv calls.
    try: # this try block ensures that the connection is closed properly even if an error occurs while handling the client.
        while True: # while loop to continuously read data from the client until the connection is closed or an error occurs.
            read_data = connection.recv(4096)
            if not read_data:
                break
            buffer += read_data.decode()
            while "\n" in buffer:
                message, buffer = buffer.split("\n", 1)
                if not message.strip():
                    continue
                try:
                    request = json.loads(message)
                except json.JSONDecodeError:
                    continue
                print(request, flush=True)
                handle_request(connection, request)

    finally:
        disconnected_calls = []
        with service_lock:
            services_to_remove = []
            for service, provider in employees.items():
                if provider is connection:
                    services_to_remove.append(service)

            for service in services_to_remove:
                del employees[service]

            for provider_id, call_info in list(forwarded_calls.items()):
                if call_info["provider"] is connection:
                    disconnected_calls.append(call_info)
                    del forwarded_calls[provider_id]

        for call_info in disconnected_calls:
            response = {
                  "op": "service_response",
                  "service": call_info["service"],
                  "id": call_info["caller_id"],
                  "values": {},
                  "result": False,
                  "status": "provider disconnected"
            }

            try:
                call_info["caller"].sendall((json.dumps(response) + "\n").encode()
                  )
            except OSError:
                  pass
                
        with topic_lock:
            for topic in list(subscribers.keys()):
                if connection in subscribers[topic]:
                    subscribers[topic].remove(connection)
            for topic in list(publishers.keys()):
                if connection in publishers[topic]:
                    publishers[topic].remove(connection)
        connection.close()
        print(f"Disconnected: {address}", flush=True)  

port = 9095
employees = {} # current provider connection for each service;
forwarded_calls = {} #forwarded calls waiting for a provider response;
next_provider_call_id = 0 # unique provider-side call IDs;
service_lock = threading.Lock() #a lock so concurrent clients do not corrupt the dictionaries
subscribers = {}
publishers ={}
topic_lock = threading.Lock() ## this lock ensures that concurrent access to the subscribers dictionary is thread-safe, preventing race conditions and data corruption when multiple threads are subscribing or unsubscribing to topics simultaneously.
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) ## sol_socket is a constant that tells the socket to use the socket layer and SO_REUSEADDR is a constant that tells the socket to reuse the address so we can restart the server without waiting for the socket to be released
server.bind((host, port))
server.listen()
print(f"Listening on {host}: {port}", flush=True)

while True:
      connection, address = server.accept()
      client_thread = threading.Thread(
          target=handle_client,
          args=(connection, address),
          daemon=True,
      )
      client_thread.start()
