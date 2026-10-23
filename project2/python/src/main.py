import socket  
import json 
import threading  
import math 
import time 
import os
import random
from integration_step import handle_integration_step
from integrators import choose_step


HOST = "127.0.0.1"
PORT = 9095

subs = {}
sub_lock = threading.Lock()
state_lock = threading.RLock()

paused = False  
current_time = 0.0 
dt = .005

num_arms = int(os.environ.get("ARM_SIM_LINKS", "2"))

if num_arms not in (2, 3):
    raise ValueError("ARM_SIM_LINKS must be 2 or 3")

current_pos = [0.0] * num_arms
current_vel = [0.0] * num_arms
intergrator_method = "rk4" 
last_effort = [0.0] * num_arms
 
 
status = False 
target_x = [0.0] * num_arms
target_v = [0.0] * num_arms
total = [0.0] * num_arms
kp = [0.0] * num_arms
ki = [0.0] * num_arms
kd = [0.0] * num_arms
 
grav = 9.81  
masses = [1.0] * num_arms
lengths = [1.0] * num_arms

ik_goal = None
ik_goal_id = None
ik_goal_counter = 0
ik_goal_epsilon = 0.05
ik_goal_success_hold = 0.0
ik_goal_inside_time = 0.0
ik_goal_start_time = 0.0

trial_running = False
trial_elapsed = 0.0
trial_duration = 30.0
trial_epsilon = 0.05
trial_start_time = 0.0
trial_success_hold = 0.0
trial_targets_reached = 0
trial_target = None
trial_error = None
trial_desired_positions = None
trial_action_status = "idle"
send_locks = {}
send_locks_lock = threading.Lock()

SAFE_MATH = {
    "sin": math.sin, "cos": math.cos, "tan": math.tan,
    "sqrt": math.sqrt, "pi": math.pi, "abs": abs
}


def _get_send_lock(conn):
    with send_locks_lock:
        lock = send_locks.get(conn)
        if lock is None:
            lock = threading.Lock()
            send_locks[conn] = lock
        return lock

def safe_send(conn, payload, timeout=0.5):
    lock = _get_send_lock(conn)
    if not lock.acquire(timeout=timeout):
        return False
    try:
        old_timeout = conn.gettimeout()
        ##conn.settimeout(timeout)
        conn.sendall(payload)
        conn.settimeout(old_timeout)
        return True
    except Exception:
        return False
    finally:
        lock.release()
        
def evaluate_checkpoint(expression, pos, vel): 
    local_vars = {} 
    for i in range(len(pos)): 
        local_vars[f"q{i+1}"] = pos[i] 
        local_vars[f"v{i+1}"] = vel[i] 
    try: 
        result = eval(expression, {"__builtins__": None, **SAFE_MATH}, local_vars) 
        return float(result) 
    except Exception as e: 
        print(f"Failed to evaluate checkpoint: {e}") 
        return None

def calculate_effort(): 
    global status, dt, current_pos, current_vel, total, kp, ki, kd, target_x, target_v
    effort = []
    if not status: 
        return [0.0] * len(current_pos)
    for i in range(len(current_pos)):
        pos_error = target_x[i] - current_pos[i]
        vel_error = target_v[i] - current_vel[i]
        total[i] += pos_error * dt
        if total[i] > 1000.0: total[i] = 1000.0
        elif total[i] < -1000.0: total[i] = -1000.0
        calc_effort = (kp[i] * pos_error) + (ki[i] * total[i]) + (kd[i] * vel_error)
        effort.append(calc_effort)    
    return effort  

def calculate_acceleration(pos, vel, tau): 
    global grav, masses, lengths 
    
    n = len(pos)
    if n == 2:
        pos1, pos2 = pos[0], pos[1]
        vel1, vel2 = vel[0], vel[1]
        m1, m2 = masses[0], masses[1]
        l1, l2 = lengths[0], lengths[1]
        
        r1, r2 = l1 / 2.0, l2 / 2.0
        
        g1 = grav * ((m1 * r1 + m2 * l1) * math.cos(pos1) + m2 * r2 * math.cos(pos1 + pos2))
        g2 = grav * (m2 * r2 * math.cos(pos1 + pos2))
        
        h = -m2 * l1 * r2 * math.sin(pos2)
        c1 = h * (2 * vel1 * vel2 + vel2**2)
        c2 = -h * (vel1**2)
        
        interia1 = (1.0/12.0) * m1 * l1**2
        interia2 = (1.0/12.0) * m2 * l2**2
        
        mm1 = m1 * r1**2 + m2 * (l1**2 + r2**2 + 2 * l1 * r2 * math.cos(pos2)) + interia1 + interia2
        mm2 = m2 * (r2**2 + l1 * r2 * math.cos(pos2)) + interia2
        m2m = mm2 
        mm22 = m2 * r2**2 + interia2
        
        rhs1 = tau[0] - c1 - g1
        rhs2 = tau[1] - c2 - g2
        
        det = (mm1 * mm22) - (mm2 * m2m)
        if abs(det) < 1e-12: return [0.0, 0.0]
        
        a1 = (mm22 * rhs1 - mm2 * rhs2) / det
        a2 = (mm1 * rhs2 - m2m * rhs1) / det
        return [a1, a2]
    
    if n == 3:
        q1, q2, q3 = pos
        v1, v2, v3 = vel
        m1, m2, m3 = masses
        l1, l2, l3 = lengths

        r1, r2, r3 = l1 / 2.0, l2 / 2.0, l3 / 2.0

        c2 = math.cos(q2)
        c3 = math.cos(q3)
        c23 = math.cos(q2 + q3)
        s2 = math.sin(q2)
        s3 = math.sin(q3)
        s23 = math.sin(q2 + q3)

        i1 = (1.0/12.0) * m1 * l1**2
        i2 = (1.0/12.0) * m2 * l2**2
        i3 = (1.0/12.0) * m3 * l3**2

        mm11 = (m1 * r1**2 + i1 + m2 * (l1**2 + r2**2 + 2 * l1 * r2 * c2) + i2
                + m3 * (l1**2 + l2**2 + r3**2
                        + 2 * l1 * l2 * c2
                        + 2 * l1 * r3 * c23
                        + 2 * l2 * r3 * c3) + i3)

        mm12 = (m2 * (r2**2 + l1 * r2 * c2) + i2+ m3 * (l2**2 + r3**2 + l1 * l2 * c2+ l1 * r3 * c23+ 2 * l2 * r3 * c3) + i3)
        mm13 = m3 * (r3**2 + l1 * r3 * c23 + l2 * r3 * c3) + i3
        mm22 = (m2 * r2**2 + i2+ m3 * (l2**2 + r3**2 + 2 * l2 * r3 * c3) + i3)
        mm23 = m3 * (r3**2 + l2 * r3 * c3) + i3
        mm33 = m3 * r3**2 + i3

        mm21 = mm12
        mm31 = mm13
        mm32 = mm23

        g1 = grav * ((m1 * r1 + m2 * l1 + m3 * l1) * math.cos(q1) + (m2 * r2 + m3 * l2) * math.cos(q1 + q2) + m3 * r3 * math.cos(q1 + q2 + q3))
        g2 = grav * ((m2 * r2 + m3 * l2) * math.cos(q1 + q2)+ m3 * r3 * math.cos(q1 + q2 + q3))
        g3 = grav * (m3 * r3 * math.cos(q1 + q2 + q3))

        k12 = l1 * l2 * (m2 + 2 * m3) * s2 + l1 * l3 * m3 * s23
        k13 = l1 * l3 * m3 * s23 + l2 * l3 * m3 * s3
        k23 = l2 * l3 * m3 * s3

        c1 = -0.5 * k12 * (2 * v1 * v2 + v2**2) - 0.5 * k13 * (2 * v1 * v3 + 2 * v2 * v3 + v3**2)
        c2 = 0.5 * k12 * v1**2 - 0.5 * k23 * (2 * v1 * v3 + 2 * v2 * v3 + v3**2)
        c3 = k23 * v1 * v2 + 0.5 * k23 * v2**2 + 0.5 * k13 * v1**2

        mm =[[mm11, mm12, mm13],
            [mm21, mm22, mm23],
            [mm31, mm32, mm33]]

        rhs = [tau[0] - c1 - g1, tau[1] - c2 - g2, tau[2] - c3 - g3]

        for i in range(3):
            pivot = i
            for j in range(i + 1, 3):
                if abs(mm[j][i]) > abs(mm[pivot][i]):
                    pivot = j

            if abs(mm[pivot][i]) < 1e-12:
                return [0.0, 0.0, 0.0]

            mm[i], mm[pivot] = mm[pivot], mm[i]
            rhs[i], rhs[pivot] = rhs[pivot], rhs[i]

            for j in range(i + 1, 3):
                factor = mm[j][i] / mm[i][i]
                for k in range(i, 3):
                    mm[j][k] -= factor * mm[i][k]
                rhs[j] -= factor * rhs[i]

        acceleration = [0.0, 0.0, 0.0]
        for i in range(2, -1, -1):
            acceleration[i] = (rhs[i] - sum(mm[i][j] * acceleration[j] for j in range(i + 1, 3))) / mm[i][i]

        return acceleration
def update_ik_action():
    global ik_goal, ik_goal_id, ik_goal_epsilon, ik_goal_success_hold
    global ik_goal_inside_time, ik_goal_start_time
    global trial_targets_reached, trial_target, trial_error
    global trial_desired_positions, trial_action_status
    global trial_running, trial_elapsed

    if ik_goal is None:
        return

    distance = calculate_target_distance(ik_goal, current_pos)
    publish_ik_feedback()

    if distance <= ik_goal_epsilon:
        if ik_goal_success_hold <= 0:
            goal_id = ik_goal_id
            publish_ik_result("reached", distance)
            ik_goal = None
            ik_goal_id = None
            ik_goal_inside_time = 0.0

            if trial_running:
                trial_targets_reached += 1
                trial_action_status = "reached"
                trial_target = None
                trial_error = distance
                trial_desired_positions = None
            current_lengths = list(lengths)
            target = sample_reachable_target(current_lengths, num_arms)
            if target is not None:
                x, y, phi = target
                sol, _ = solve_ik_positions(x, y, phi, current_lengths, num_arms)
                activate_goal(x, y, phi, trial_epsilon, trial_success_hold, sol)
                trial_target = dict(ik_goal)
                trial_desired_positions = list(sol)
                trial_error = calculate_target_distance(ik_goal, current_pos)
                trial_action_status = "active"

        else:
            ik_goal_inside_time += dt
            if ik_goal_inside_time >= ik_goal_success_hold:
                goal_id = ik_goal_id
                publish_ik_result("reached", distance)
                ik_goal = None
                ik_goal_id = None
                ik_goal_inside_time = 0.0

                if trial_running:
                    trial_targets_reached += 1
                    trial_action_status = "reached"
                    trial_target = None
                    trial_error = distance
                    trial_desired_positions = None
                current_lengths = list(lengths)
                target = sample_reachable_target(current_lengths, num_arms)
                if target is not None:
                    x, y, phi = target
                    sol, _ = solve_ik_positions(x, y, phi, current_lengths, num_arms)
                    activate_goal(x, y, phi, trial_epsilon, trial_success_hold, sol)
                    trial_target = dict(ik_goal)
                    trial_desired_positions = list(sol)
                    trial_error = calculate_target_distance(ik_goal, current_pos)
                    trial_action_status = "active"
    else:
        ik_goal_inside_time = 0.0   
    
def physics_tick():  
    global paused, current_time, dt, current_pos, current_vel, intergrator_method, last_effort  
    if paused:  
        return  
          
    tau = calculate_effort()  
    
    def acceleration(step_pos, step_vel, step_t): 
        return calculate_acceleration(step_pos, step_vel, tau) 
 
    new_pos, new_vel = choose_step(intergrator_method, acceleration, current_time, current_pos, current_vel, dt)  
    current_pos = new_pos  
    current_vel = new_vel  
    current_time += dt  
    last_effort = tau

    update_ik_action()

def run_physics():
    global trial_running, trial_start_time, trial_duration, trial_targets_reached
    global trial_target, trial_error, trial_desired_positions, trial_action_status
    global ik_goal, ik_goal_id
    print("Physics engine started.")
    
    while True:
        with state_lock:
            physics_tick()

            if trial_running and (current_time - trial_start_time) >= trial_duration:
                if ik_goal is not None:
                    final_distance = calculate_target_distance(ik_goal, current_pos)
                    publish_ik_result("preempted", final_distance)
                    ik_goal = None
                    ik_goal_id = None
                trial_elapsed = current_time - trial_start_time
                trial_running = False
                trial_target = None
                trial_error = None
                trial_desired_positions = None
                trial_action_status = "idle"
                publish_trial_status()

            if trial_running or trial_action_status != "idle":
                status_payload = {
                    "running": trial_running,
                    "elapsed": current_time - trial_start_time if trial_running else trial_elapsed,
                    "duration": trial_duration,
                    "targets_reached": trial_targets_reached,
                    "target": trial_target,
                    "error": trial_error,
                    "desired_positions": trial_desired_positions,
                    "action_status": trial_action_status,
                }
                publish_to_subscribers("/ik_trial/status", status_payload)
                
            current_sec = int(current_time)
            current_nanosec = int((current_time - current_sec) * 1000000000)

            state_msg = { 
                "op": "publish", 
                "topic": "/joint_states", 
                "msg": { 
                    "header": {
                        "stamp": {
                            "sec": current_sec,
                            "nanosec": current_nanosec
                        },
                        "frame_id": ""
                    }, 
                    "name": [f"joint{i+1}" for i in range(len(current_pos))], 
                    "position": list(current_pos), 
                    "velocity": list(current_vel), 
                    "effort": list(last_effort), 
                }, 
            }
            current_dt = dt

        payload = (json.dumps(state_msg) + "\n").encode('utf-8')
        
        with sub_lock:
            subscribers = list(subs.get("/joint_states", []))

        dead_connections = []
        for conn in subscribers:
           if not safe_send(conn, payload):
                dead_connections.append(conn)

        if dead_connections:
            with sub_lock:
                if "/joint_states" in subs:
                    for conn in dead_connections:
                        if conn in subs["/joint_states"]:
                            subs["/joint_states"].remove(conn)

        time.sleep(current_dt)


def send_message(connection, message):
    connection.sendall((json.dumps(message) + "\n").encode('utf-8'))


def publish_to_subscribers(topic, msg):
    publish_packet = {
        "op": "publish",
        "topic": topic,
        "msg": msg
    }
    payload = (json.dumps(publish_packet) + "\n").encode('utf-8')

    with sub_lock:
        subscribers = list(subs.get(topic, []))

    dead_connections = []
    for conn in subscribers:
        if not safe_send(conn, payload):
            dead_connections.append(conn)

    if dead_connections:
        with sub_lock:
            if topic in subs:
                for conn in dead_connections:
                    if conn in subs[topic]:
                        subs[topic].remove(conn)

def publish_ik_feedback():
    global ik_goal, ik_goal_id, ik_goal_epsilon, ik_goal_success_hold
    global current_pos, current_time

    if ik_goal is None:
        return

    distance = calculate_target_distance(ik_goal, current_pos)
    feedback = {
        "goal_id": ik_goal_id,
        "target": dict(ik_goal),
        "positions": list(target_x),
        "distance_remaining": distance,
        "elapsed": current_time - ik_goal_start_time
    }
    publish_to_subscribers("/ik_action/feedback", feedback)

def publish_ik_result(outcome, final_distance):
    global ik_goal, ik_goal_id

    if ik_goal is None or ik_goal_id is None:
        return
    result = {
        "goal_id": ik_goal_id,
        "outcome": outcome,
        "target": dict(ik_goal),
        "final_distance": final_distance
    }
    publish_to_subscribers("/ik_action/result", result)
def publish_trial_status():
    payload = {
        "running": trial_running,
        "elapsed": (current_time - trial_start_time) if trial_running else trial_elapsed,
        "duration": trial_duration,
        "targets_reached": trial_targets_reached,
        "target": trial_target,
        "error": trial_error,
        "desired_positions": trial_desired_positions,
        "action_status": trial_action_status,
    }
    publish_to_subscribers("/ik_trial/status", payload)   
    
def calculate_end_effector(pos):
    if len(pos) == 2:
        q1, q2 = pos
        x = lengths[0] * math.cos(q1) + lengths[1] * math.cos(q1 + q2)
        y = lengths[0] * math.sin(q1) + lengths[1] * math.sin(q1 + q2)
        return x, y, q1 + q2

    q1, q2, q3 = pos
    x = (lengths[0] * math.cos(q1) 
         + lengths[1] * math.cos(q1 + q2) 
         + lengths[2] * math.cos(q1 + q2 + q3))
    y = (lengths[0] * math.sin(q1) 
         + lengths[1] * math.sin(q1 + q2) 
         + lengths[2] * math.sin(q1 + q2 + q3))
    return x, y, q1 + q2 + q3

def calculate_target_distance(target, pos):
    x, y, phi = calculate_end_effector(pos)

    distance = math.sqrt((target["x"] - x)**2 + (target["y"] - y)**2)
    return distance
def solve_ik_positions(x, y, phi, current_lengths, arms):  
    eps = 1e-9
    if arms == 2:
        l1, l2 = current_lengths
        distance_sq = x**2 + y**2
        if distance_sq > (l1 + l2)**2 + eps or distance_sq < (l1 - l2)**2 - eps:
            return None, "Unreachable point"
        cos_q2 = max(-1.0, min(1.0, (distance_sq - l1**2 - l2**2) / (2 * l1 * l2)))
        q2 = math.acos(cos_q2)
        q1 = math.atan2(y, x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        return [q1, q2], ""

    l1, l2, l3 = current_lengths
    phi_use = 0.0 if phi is None else phi
    wrist_x = x - l3 * math.cos(phi_use)
    wrist_y = y - l3 * math.sin(phi_use)
    wrist_sq = wrist_x**2 + wrist_y**2
    if wrist_sq > (l1 + l2)**2 + eps or wrist_sq < (l1 - l2)**2 - eps:
        return None, "Unreachable point"
    cos_q2 = max(-1.0, min(1.0, (wrist_sq - l1**2 - l2**2) / (2 * l1 * l2)))
    q2 = math.acos(cos_q2)
    q1 = math.atan2(wrist_y, wrist_x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    q3 = phi_use - q1 - q2
    return [q1, q2, q3], ""


def sample_reachable_target(current_lengths, arms, max_tries=200):
    total = sum(current_lengths)
    for _ in range(max_tries):
        x = random.uniform(-total, total)
        y = random.uniform(-total, total)
        phi = random.uniform(-math.pi, math.pi) if arms == 3 else None
        sol, status = solve_ik_positions(x, y, phi, current_lengths, arms)
        if sol is not None:
            return x, y, phi
    return None


def activate_goal(x, y, phi, epsilon, success_hold, positions):
    # caller must already hold state_lock
    global ik_goal, ik_goal_id, ik_goal_counter, ik_goal_epsilon
    global ik_goal_success_hold, ik_goal_inside_time, ik_goal_start_time
    global target_x, target_v

    if ik_goal is not None:
        final_distance = calculate_target_distance(ik_goal, current_pos)
        publish_ik_result("preempted", final_distance)

    ik_goal_counter += 1
    new_goal_id = str(ik_goal_counter)

    ik_goal = {"x": x, "y": y}
    if num_arms == 3 and phi is not None:
        ik_goal["phi"] = phi

    ik_goal_id = new_goal_id
    ik_goal_epsilon = epsilon
    ik_goal_success_hold = success_hold
    ik_goal_inside_time = 0.0
    ik_goal_start_time = current_time

    target_x = list(positions)
    target_v = [0.0] * num_arms
    return new_goal_id

def handle_client(connection, address):
    global paused, status, total, kp, ki, kd, grav, masses, lengths
    global intergrator_method, dt, current_pos, current_vel, current_time, target_x, target_v
    global ik_goal, ik_goal_id, ik_goal_counter, ik_goal_epsilon, ik_goal_success_hold
    global ik_goal_inside_time, ik_goal_start_time, last_effort
    global trial_running, trial_elapsed, trial_duration, trial_epsilon, trial_success_hold
    global trial_targets_reached, trial_target, trial_error, trial_desired_positions, trial_action_status, trial_start_time

    print(f"Connected through {address}")
    try:
        with connection, connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                line = line.strip()
                if not line: continue
                try: request = json.loads(line)
                except json.JSONDecodeError:
                    continue
                print(f"Received: {request}", flush=True)   # ADD THIS LINE
                op = request.get("op")
                
                if op == "call_service":
                    service_name = request.get("service")
                    args = request.get("args", {})
                    req_id = request.get("id")
                    
                    response = { 
                        "op": "service_response", "service": service_name, 
                        "id": req_id, "values": {}, "result": True, "status": ""
                    }
                    
                    if service_name == "/arm_sim/pause":
                        new_paused = args.get("data", paused)
                        if isinstance(new_paused, bool):
                            with state_lock:
                                paused = new_paused
                            response["values"] = {"data": paused}
                        else:
                            response["result"] = False
                            response["status"] = "data must be boolean"
                        send_message(connection, response)
                        
                    elif service_name == "/pid_controller/enable":
                        new_status = args.get("data", status)
                        if isinstance(new_status, bool):
                            with state_lock:
                                status = new_status
                                if status == False: total = [0.0] * len(current_pos)
                            response["values"] = {"data": status}
                        else:
                            response["result"] = False
                            response["status"] = "data must be boolean"
                        send_message(connection, response)
                        
                    elif service_name == "/pid_controller/set_gains":
                        with state_lock:
                            for name, cur in (("kp", kp), ("ki", ki), ("kd", kd)):
                                if name in args:
                                    candidate = args[name]
                                    if isinstance(candidate, list) and len(candidate) == len(cur) and all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in candidate):
                                        if name == "kp": kp = list(candidate)
                                        elif name == "ki": ki = list(candidate)
                                        else: kd = list(candidate)
                                    else:
                                        response["result"] = False
                                        response["status"] = f"invalid {name}"
                            response["values"] = {"kp": kp, "ki": ki, "kd": kd}
                        send_message(connection, response)
                            
                    elif service_name == "/arm_sim/set_params":
                        with state_lock:
                            if "gravity" in args:
                                g_new = args["gravity"]
                                if isinstance(g_new, (int, float)) and math.isfinite(g_new) and g_new >= 0:
                                    grav = g_new
                                else:
                                    response["result"] = False
                                    response["status"] = "invalid gravity"
                            if "masses" in args:
                                m_new = args["masses"]
                                if isinstance(m_new, list) and len(m_new) == len(masses) and all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in m_new):
                                    masses = list(m_new)
                                else:
                                    response["result"] = False
                                    response["status"] = "invalid masses"
                            if "lengths" in args:
                                l_new = args["lengths"]
                                if isinstance(l_new, list) and len(l_new) == len(lengths) and all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in l_new):
                                    lengths = list(l_new)
                                else:
                                    response["result"] = False
                                    response["status"] = "invalid lengths"
                            response["values"] = {"gravity": grav, "masses": masses, "lengths": lengths}
                        send_message(connection, response)
                        
                    elif service_name == "/arm_sim/set_integrator":
                        new_method = args.get("method", intergrator_method)
                        new_dt = args.get("timestep", dt)
                        if new_method not in ("euler", "midpoint", "verlet", "rk4") or not isinstance(new_dt, (int, float)) or not math.isfinite(new_dt) or new_dt <= 0:
                            response["result"] = False
                            response["status"] = "invalid method or non-positive timestep"
                        else:
                            with state_lock:
                                intergrator_method, dt = new_method, new_dt
                            response["values"] = {"method": intergrator_method, "timestep": dt}
                        send_message(connection, response)
                            
                    elif service_name == "/arm_sim/reset": 
                        with state_lock: 
                            current_pos = [0.0] * num_arms
                            current_vel = [0.0] * num_arms
                            current_time = 0.0 
                            target_v = [0.0] * num_arms
                            target_x = [0.0] * num_arms
                            total = [0.0] * num_arms
                            last_effort = [0.0] * num_arms
                            response["values"] = {"position": list(current_pos), "velocity": list(current_vel)} 
                        send_message(connection, response)

                    elif service_name == "/arm_sim/checkpoint":
                        expr_string = args.get("expr", "")
                        with state_lock:
                            pos_copy = list(current_pos)
                            vel_copy = list(current_vel)
                        val = evaluate_checkpoint(expr_string, pos_copy, vel_copy)
                        response["values"], response["result"] = {"val": val}, (val is not None)
                        send_message(connection, response)
                        
                    elif service_name == "/arm_sim/integration_step":
                        result = handle_integration_step(args)
                        response["values"] = result["values"]
                        response["result"] = result["result"]
                        response["status"] = result["status"]
                        send_message(connection, response)
                        
        
                    
                    elif service_name == "/ik_action/send_goal":
                        if "x" not in args or "y" not in args or not isinstance(args.get("x"), (int, float)) or not isinstance(args.get("y"), (int, float)) or not math.isfinite(args.get("x")) or not math.isfinite(args.get("y")):
                            response["result"] = False
                            response["status"] = "missing numeric x or y"
                            send_message(connection, response)
                            continue

                        x, y = args["x"], args["y"]
                        phi = args.get("phi") if num_arms == 3 else None
                        if phi is not None and (not isinstance(phi, (int, float)) or not math.isfinite(phi)):
                            response["result"] = False
                            response["status"] = "invalid phi"
                            send_message(connection, response)
                            continue

                        epsilon = args.get("epsilon", 0.05)
                        success_hold = args.get("success_hold", 0.0)
                        if not isinstance(epsilon, (int, float)) or not math.isfinite(epsilon) or epsilon <= 0:
                            response["result"] = False
                            response["status"] = "invalid epsilon"
                            send_message(connection, response)
                            continue
                        if not isinstance(success_hold, (int, float)) or not math.isfinite(success_hold) or success_hold < 0:
                            response["result"] = False
                            response["status"] = "invalid success_hold"
                            send_message(connection, response)
                            continue

                        with state_lock:
                            current_lengths = list(lengths)

                        sol, status_msg = solve_ik_positions(x, y, phi, current_lengths, num_arms)
                        if sol is None:
                            response["result"] = False
                            response["status"] = status_msg
                            send_message(connection, response)
                            continue

                        with state_lock:
                            new_goal_id = activate_goal(x, y, phi, epsilon, success_hold, sol)

                        response["values"] = {"goal_id": new_goal_id}
                        send_message(connection, response)

                    elif service_name == "/ik_action/cancel_goal":
                        goal_id = args.get("goal_id")
                        with state_lock:
                            if ik_goal is None:
                                response["result"] = False
                                response["status"] = "no active goal"
                            elif goal_id is not None and goal_id != ik_goal_id:
                                response["result"] = False
                                response["status"] = "goal_id does not match active goal"
                            else:
                                final_distance = calculate_target_distance(ik_goal, current_pos)
                                publish_ik_result("preempted", final_distance)
                                ik_goal = None
                                ik_goal_id = None
                                ik_goal_inside_time = 0.0
                        send_message(connection, response)

                    elif service_name == "/ik_trial/start":
                        duration = args.get("duration", trial_duration)
                        t_epsilon = args.get("epsilon", trial_epsilon)
                        t_success_hold = args.get("success_hold", trial_success_hold)
                        with state_lock:
                            current_lengths = list(lengths)
                            target = sample_reachable_target(current_lengths, num_arms)
                            if target is None:
                                response["result"] = False
                                response["status"] = "could not sample a reachable target"
                                send_message(connection, response)
                                continue
                            x, y, phi = target
                            sol, _ = solve_ik_positions(x, y, phi, current_lengths, num_arms)

                            trial_running = True
                            trial_elapsed = 0.0
                            trial_start_time = current_time
                            trial_duration = duration
                            trial_epsilon = t_epsilon
                            trial_success_hold = t_success_hold
                            trial_targets_reached = 0
                            activate_goal(x, y, phi, t_epsilon, t_success_hold, sol)
                            trial_target = dict(ik_goal)
                            trial_desired_positions = list(sol)
                            trial_error = calculate_target_distance(ik_goal, current_pos)
                            trial_action_status = "active"
                        publish_trial_status()
                        send_message(connection, response)

                    elif service_name == "/ik_trial/skip":
                        with state_lock:
                            if not trial_running:
                                response["result"] = False
                                response["status"] = "no trial running"
                                send_message(connection, response)
                                continue
                            if ik_goal is not None:
                                final_distance = calculate_target_distance(ik_goal, current_pos)
                                publish_ik_result("preempted", final_distance)
                                ik_goal = None
                                ik_goal_id = None
                                ik_goal_inside_time = 0.0
                            current_lengths = list(lengths)
                            target = sample_reachable_target(current_lengths, num_arms)
                            if target is None:
                                response["result"] = False
                                response["status"] = "could not sample a reachable target"
                                send_message(connection, response)
                                continue
                            x, y, phi = target
                            sol, _ = solve_ik_positions(x, y, phi, current_lengths, num_arms)
                            activate_goal(x, y, phi, trial_epsilon, trial_success_hold, sol)
                            trial_target = dict(ik_goal)
                            trial_desired_positions = list(sol)
                            trial_error = calculate_target_distance(ik_goal, current_pos)
                            trial_action_status = "active"
                        publish_trial_status()
                        send_message(connection, response)

                    elif service_name == "/ik_trial/stop":
                        with state_lock:
                            if not trial_running:
                                response["result"] = False
                                response["status"] = "no trial running"
                                send_message(connection, response)
                                continue
                            if ik_goal is not None:
                                final_distance = calculate_target_distance(ik_goal, current_pos)
                                publish_ik_result("preempted", final_distance)
                                ik_goal = None
                                ik_goal_id = None
                                ik_goal_inside_time = 0.0
                            trial_elapsed = current_time - trial_start_time
                            trial_running = False
                            trial_target = None
                            trial_error = None
                            trial_desired_positions = None
                            trial_action_status = "idle"
                        publish_trial_status() 
                        send_message(connection, response)
                        
                    elif service_name == "/ik/solve": 
                        if "x" not in args or "y" not in args or not isinstance(args.get("x"), (int, float)) or not isinstance(args.get("y"), (int, float)) or not math.isfinite(args.get("x")) or not math.isfinite(args.get("y")):
                            response["result"], response["status"] = False, "missing numeric x or y"
                            send_message(connection, response)
                            continue

                        x, y = args["x"], args["y"]
                        phi = args.get("phi") if num_arms == 3 else None
                        if phi is not None and (not isinstance(phi, (int, float)) or not math.isfinite(phi)):
                            response["result"], response["status"] = False, "invalid phi"
                            send_message(connection, response)
                            continue

                        with state_lock:
                            current_lengths = list(lengths)

                        sol, status_msg = solve_ik_positions(x, y, phi, current_lengths, num_arms)
                        if sol is None:
                            response["result"], response["status"] = False, status_msg
                        else:
                            response["values"] = {"positions": sol}
                        send_message(connection, response)
                         
                    else:
                        response["result"] = False
                        response["status"] = f"unknown service '{service_name}'"
                        send_message(connection, response)
 
                elif op == "subscribe":
                    topic = request.get("topic")
                    if not topic or "type" not in request:
                        send_message(connection, {"op": "status", "level": "error",
                                                "msg": "subscribe requires topic and type", "id": request.get("id")})
                        continue
                    with sub_lock:
                        subs.setdefault(topic, [])
                        if connection not in subs[topic]:
                            subs[topic].append(connection)
                    status_msg = {"op": "status", "level": "info",
                                "msg": f"subscribed to {topic}", "id": request.get("id")}
                    send_message(connection, status_msg)

                elif op == "unsubscribe":
                    topic = request.get("topic")
                    if not topic:
                        send_message(connection, {"op": "status", "level": "error",
                                                "msg": "unsubscribe requires topic", "id": request.get("id")})
                        continue
                    with sub_lock:
                        if topic in subs and connection in subs[topic]:
                            subs[topic].remove(connection)
                    status_msg = {"op": "status", "level": "info",
                                  "msg": f"unsubscribed from {topic}", "id": request.get("id")}
                    send_message(connection, status_msg)

                elif op == "publish": 
                    topic = request.get("topic")
                    msg = request.get("msg", {})
                    
                    if topic == "/joint_trajectory":
                        points = msg.get("points", [])
                        if isinstance(points, list) and len(points) > 0:
                            last_point = points[-1]
                            positions = last_point.get("positions")
                            velocities = last_point.get("velocities")
                            with state_lock:
                                if isinstance(positions, list) and len(positions) == num_arms and all(isinstance(v, (int, float)) and math.isfinite(v) for v in positions):
                                    target_x = list(positions)
                                else:
                                    target_x = [0.0] * num_arms
                                if isinstance(velocities, list) and len(velocities) == num_arms and all(isinstance(v, (int, float)) and math.isfinite(v) for v in velocities):
                                    target_v = list(velocities)
                                else:
                                    target_v = [0.0] * num_arms

                    publish_to_subscribers(topic, msg)
                elif op == "advertise":
                    pass
                elif op == "unadvertise":
                    pass
                elif op == "advertise_service":
                    status_msg = {"op": "status", "level": "info",
                                "msg": f"advertised {request.get('service')}", "id": request.get("id")}
                    send_message(connection, status_msg)
                elif op == "unadvertise_service":
                    pass
                elif op == "service_response":
                    pass

                else:
                    req_id = request.get("id")
                    send_message(connection, {
                        "op": "status",
                        "level": "error",
                        "msg": f"unknown op '{op}'",
                        "id": req_id
                    })
                             
    except Exception as e:
        print(f"Connection error: {e}")
            
    with sub_lock:
        for conn_list in subs.values():
            if connection in conn_list:
                conn_list.remove(connection)
    print(f"Disconnected: {address}")

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    print(f"Rosbridge Gateway listening on {HOST}:{PORT}")
    print("Rosbridge Gateway ready")
    physics_thread = threading.Thread(target=run_physics, daemon=True)
    physics_thread.start()
    
    while True:
        connection, addy = server.accept()
        client_thread = threading.Thread(target=handle_client, args=(connection, addy), daemon=True)
        client_thread.start()

if __name__ == "__main__":
    start_server()
