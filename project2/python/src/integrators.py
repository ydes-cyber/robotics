#integrators.py
def euler_step(acceleration, t, position, velocity, dt):
    # Ask the acceleration function for the acceleration at the current time.
    a = acceleration(position, velocity, t)
    
    # updates position with the v at the beginning of this step.
    current_x = [position[i] + velocity[i] * dt for i in range(len(position))]
    
    # Acc changes velocity over the duration of the timestep.
    current_v = [velocity[i] + a[i] * dt for i in range(len(velocity))]
    
    # Give the caller the updated position first, then updated velocity.
    return current_x, current_v

def midpoint_step(acceleration, t, position, velocity, dt):
    # get the acc at the start
    a = acceleration(position, velocity, t)
    
    # get a halfway guess
    mid_pos = [position[i] + velocity[i] * (dt/2.0) for i in range(len(position))]
    mid_vel = [velocity[i] + a[i] * (dt/2.0) for i in range(len(velocity))]
    
    # get the acc halfway through
    mid_a = acceleration(mid_pos, mid_vel, t + dt/2.0)
    
    # use the halfway values for the whole step
    new_x = [position[i] + mid_vel[i] * dt for i in range(len(position))]
    new_v = [velocity[i] + mid_a[i] * dt for i in range(len(velocity))]
    
    return new_x, new_v

def verlet_step(acceleration, t, position, velocity, dt):
    # get the starting acc 
    a_start = acceleration(position, velocity, t)
    
    # use start acc to get the position change
    verlet_x = [position[i] + velocity[i] * dt + (a_start[i] * dt**2 / 2.0) for i in range(len(position))]
    
    # get a guess for the velocity at the end
    v_pred = [velocity[i] + a_start[i] * dt for i in range(len(velocity))]
    a_end = acceleration(verlet_x, v_pred, t + dt)
    
    # use the avg acc to get the final velocity
    verlet_v = [velocity[i] + ((a_start[i] + a_end[i]) / 2.0) * dt for i in range(len(velocity))]
    
    return verlet_x, verlet_v

def rk4_step(acceleration, t, position, velocity, dt):
    k1x = velocity 
    k1v = acceleration(position, velocity, t)
   
    pos2 = [position[i] + k1x[i] * (dt / 2.0) for i in range(len(position))]
    vel2 = [velocity[i] + k1v[i] * (dt / 2.0) for i in range(len(velocity))]
    k2x = vel2
    k2v = acceleration(pos2, vel2, t + dt / 2.0)
    
    pos3 = [position[i] + k2x[i] * (dt / 2.0) for i in range(len(position))]
    vel3 = [velocity[i] + k2v[i] * (dt / 2.0) for i in range(len(velocity))]
    k3x = vel3
    k3v = acceleration(pos3, vel3, t + dt / 2.0)
        
    pos4 = [position[i] + k3x[i] * dt for i in range(len(position))]
    vel4 = [velocity[i] + k3v[i] * dt for i in range(len(velocity))]
    k4x = vel4
    k4v = acceleration(pos4, vel4, t + dt)
    
    new_x = [position[i] + dt/6.0 * (k1x[i] + 2*k2x[i] + 2*k3x[i] + k4x[i]) for i in range(len(position))]
    new_v = [velocity[i] + dt/6.0 * (k1v[i] + 2*k2v[i] + 2*k3v[i] + k4v[i]) for i in range(len(velocity))]
    
    return new_x, new_v 

class UnknownIntegratorError(ValueError):
    pass

_METHODS = {
    "euler": euler_step,
    "midpoint": midpoint_step,
    "verlet": verlet_step,
    "rk4": rk4_step,
}

def choose_step(method_name, acceleration, t, position, velocity, dt):
    step_fn = _METHODS.get(method_name)
    if step_fn is None:
        raise UnknownIntegratorError(f"unknown integrator '{method_name}'")
    return step_fn(acceleration, t, position, velocity, dt)