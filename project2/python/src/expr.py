#expr.py
import math

SAFE_MATH = {
    "sin": math.sin, "cos": math.cos, "tan": math.tan,
    "sqrt": math.sqrt, "pi": math.pi, "abs": abs
}

def evaluate_checkpoint(expression, pos, vel):
    local_vars = {"q1": pos[0], "q2": pos[1], "v1": vel[0], "v2": vel[1]}
    try:
        result = eval(expression, {"__builtins__": None, **SAFE_MATH}, local_vars)
        return float(result)
    except Exception as e:
        print(f"Failed to evaluate checkpoint: {e}")
        return None