import numpy as np

def upd_pos_vel(Ncar, pos, vel, acc, headway, dv, posnew, velnew, params):

    # Extract parameters
    des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t = params[:7]

    # Loop over all cars
    for i in range(Ncar):

        # Calculate desired bumper-to-bumper distance (s*)
        s_star = min_gap + max(0, vel[i] * time_gap + (vel[i] * dv[i]) / (2 * (acc_max * comf_decel)**0.5))

        # Calculate acceleration using IDM
        acc[i] = acc_max * (1 - (vel[i] * des_speed_inv)**acc_exp - (s_star / headway[i])**2)

        # Update velocity and position using Newtonian dynamics
        velnew[i] = vel[i] + acc[i] * del_t
        posnew[i] = pos[i] + vel[i] * del_t + 0.5 * acc[i] * del_t**2

        # Ensure velocity does not go negative
        if velnew[i] < 0:

            # Calculate time to stop
            t_stop = -vel[i] / acc[i]

            # Update position and velocity to stop at t_stop
            posnew[i] = pos[i] + vel[i] * t_stop + 0.5 * acc[i] * t_stop**2
            velnew[i] = 0

    return posnew, velnew, acc

def detect_loop(Ncar, pos, vel, acc, posnew, velnew, detect_time, detect_vel, det_point, time_pass):

    for i in range(Ncar):

        # Check if the car has passed the detection point
        if pos[i] < det_point <= posnew[i]:

            # Linear interpolation to find the exact time and speed
            delta_t = (det_point - pos[i]) / vel[i]
            detect_time[i] = time_pass + delta_t
            detect_vel[i] = vel[i] + acc[i] * delta_t

    return detect_time, detect_vel

def flow_global(N, vel, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.mean(vel) * dens * 3600 / 1000

    return dens, flow

def update_cars(Ncar, pos, vel, posnew, velnew, headway, dv, params):

    # Extract the key parameters from params
    length, L, min_gap  = params[7], params[8], params[4]

    # Update positions and velocities
    for i in range(Ncar):
        pos[i] = posnew[i] % L
        vel[i] = velnew[i]

    # Update headway and velocity difference
    for i in range(Ncar):
        next_car = (i + 1) % Ncar

        # Calculate headway, accounting for the length of the cars
        headway[i] = (pos[next_car] - pos[i] - length) % L

        # Ensure the minimum gap is maintained
        if headway[i] < min_gap:
            headway[i] = min_gap

            # Adjust the position of the current car to maintain the minimum gap
            pos[i] = (pos[next_car] - min_gap - length) % L
            
        # Update velocity difference
        dv[i] = vel[next_car] - vel[i]

    return pos, vel, headway, dv

def Step(N, pos, vel, headway, dv, params, time_pass, time_measure, detect_time, detect_vel, det_point, L):
    
    # Initialise arrays
    posnew = np.zeros(N)
    velnew = np.zeros(N)
    acc = np.zeros(N)
    den = 0
    flo = 0

    # Determine the position, velocity at end of interval and acceleration at start
    posnew, velnew, acc = upd_pos_vel(N, pos, vel, acc, headway, dv, posnew, velnew, params)

    # Second phase: After a certain number of steps, activate the detection loop
    if time_pass > time_measure:

        # Calculate global flow and density
        den, flo = flow_global(N, velnew, L)

        # Detection loop for local measurements
        detect_time, detect_vel = detect_loop(N, pos, vel, acc, posnew, velnew, detect_time, detect_vel, det_point, time_pass)

    # Update the key parameters needed for the next iteration
    pos, vel, headway, dv = update_cars(N, pos, vel, posnew, velnew, headway, dv, params)

    return pos, vel, headway, dv, den, flo, detect_time, detect_vel

def init_params():

    # Model parameters
    Nmax = 100  # Maximum number of cars
    L = 10000  # Length of ring road (meters)
    steps = 1000  # Total number of steps
    steps_measure = 100  # Steps before we start to measure
    splim = 30  # Speed limit in m/s (approx 108 km/h)
    des_speed = splim  # Desired speed (m/s)
    des_speed_inv = 1.0 / splim  # Inverse of desired speed
    del_t = 0.5  # Time step in seconds
    acc_exp = 4  # Acceleration exponent
    time_gap = 1.0  # Time gap in seconds
    min_gap = 2.0  # Minimum spatial gap in meters
    comf_decel = 1.5  # Comfortable deceleration in m/s^2
    acc_max = 1.0  # Maximum acceleration in m/s^2
    length = 3.0  # Car length in meters
    det_point = L / 2  # Detection point in meters

    # Params list setup
    params = [des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t, length, L]

    return Nmax, L, steps, steps_measure, splim, params, det_point

def init_simulation(N,L, params):

    # Initial conditions
    vel = np.zeros(N) 
    pos = np.zeros(N)

    # Check if the road can accommodate all cars with the required spacing
    total_space_per_car = params[7] + params[4]
    if N * total_space_per_car > L:
        raise ValueError(f"Cannot fit {N} cars on a road of length {L} with min_gap={params[4]} and car length={params[7]}.")

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (params[7] + params[4])

    dv = np.zeros(N) 

    # Calculate headway
    headway = np.zeros(N)
    for i in range(N):
        next_car = (i + 1) % N
        headway[i] = (pos[next_car] - pos[i] - params[7]) % L

    print(N, 'Cars initialised')
    
    return vel, pos, dv, headway

def analyse_global(track_flow, track_dens):

    # Calculate overall global flow and density
    glob_flow = np.mean(track_flow)
    glob_dens = np.mean(track_dens)
    
    return glob_flow, glob_dens

def analyse_local(track_det_time, track_det_vel):
    
    # No data yet
    if len(track_det_time) == 0 or len(track_det_vel) == 0:
        return 0, 0

    # Calculate local flow: Average velocity at the detection point (cars/hour)
    loc_flow = np.mean(track_det_vel) * 3600 / 1000

    # Calculate local density: Number of cars passing per unit time (cars/km)
    time_window = max(track_det_time) - min(track_det_time)

    # Avoid division by zero
    if time_window == 0:
        return 0, 0 
    
    loc_dens = len(track_det_time) / (time_window * 3600 / 1000)

    return loc_flow, loc_dens

def Simulate_IDM(N, params, steps, steps_measure,det_point):
    L = params[8]
    del_t = params[6]
    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []
    detect_time = np.zeros(N)
    detect_vel = np.zeros(N)

    vel, pos, dv, headway = init_simulation(N, L, params)

    for i in range(steps):
        time_pass = i * del_t

        if time_pass > steps_measure * del_t:
            pos, vel, headway, dv, den, flo, detect_time, detect_vel = Step(N, pos, vel, headway, dv, params, time_pass, steps_measure * del_t, detect_time, detect_vel, det_point, L)
            
            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            pos, vel, headway, dv, den, flo, _, _ = Step(N, pos, vel, headway, dv, params, time_pass, steps_measure * del_t, [], [], det_point, L)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel)

    print('Simulation for car total', N, 'completed')

    return glob_flow, glob_dens, loc_flow, loc_dens