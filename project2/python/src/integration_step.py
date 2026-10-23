from math_step import make_forcing_function, ExprError
from integrators import choose_step, UnknownIntegratorError

VALID_INTEGRATORS = {"euler", "midpoint", "verlet", "rk4"}

def reject_request(status):
    return {"values": {}, "result": False, "status": status}

def handle_integration_step(args):
    if not isinstance(args, dict):
        return reject_request("args must be an object")

    function_str = args.get("function")
    if not isinstance(function_str, str) or function_str == "":
        return reject_request("'function' must be a non-empty string")

    if "x0" not in args:
        return reject_request("'x0' is required")
    try:
        x0 = float(args["x0"])
    except (TypeError, ValueError):
        return reject_request("'x0' must be a number")

    try:
        xdot0 = float(args.get("xdot0", 0.0))
    except (TypeError, ValueError):
        return reject_request("'xdot0' must be a number")

    if "dt" not in args:
        return reject_request("'dt' is required")
    try:
        dt = float(args["dt"])
    except (TypeError, ValueError):
        return reject_request("'dt' must be a number")
    if dt <= 0:
        return reject_request("'dt' must be > 0")

    if "steps" not in args:
        return reject_request("'steps' is required")
    steps_value = args["steps"]
    if isinstance(steps_value, bool) or not isinstance(steps_value, int):
        return reject_request("'steps' must be a non-negative integer")
    if steps_value <= 0:
        return reject_request("'steps' must be > 0")
    steps = steps_value

    integrator_name = args.get("integrator")
    if integrator_name not in VALID_INTEGRATORS:
        return reject_request(f"'integrator' must be one of {sorted(VALID_INTEGRATORS)}")

    try:
        forcing_function = make_forcing_function(function_str)
        forcing_function(0.0)
    except ExprError as error:
        return reject_request(f"could not parse 'function': {error}")
    except Exception as error:
        return reject_request(f"'function' failed to evaluate: {error}")

    def acceleration(position, velocity, time):
        return [forcing_function(time)]

    times = [0.0]
    positions = [x0]
    velocities = [xdot0]

    position = [x0]
    velocity = [xdot0]
    current_time = 0.0

    try:
        for step in range(steps):
            position, velocity = choose_step(
                integrator_name,
                acceleration,
                current_time,
                position,
                velocity,
                dt
            )
            current_time += dt
            times.append(current_time)
            positions.append(position[0])
            velocities.append(velocity[0])
    except UnknownIntegratorError as error:
        return reject_request(str(error))

    return {
        "values": {
            "times": times,
            "positions": positions,
            "velocities": velocities
        },
        "result": True,
        "status": ""
    }