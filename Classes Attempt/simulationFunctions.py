import numpy as np
from VehicleClass import VehicleClass as vc

def flow_global(N, velnew, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.mean(velnew) * dens * 3600 / 1000

    return dens, flow

def Step(N, cars, params, time_pass, time_measure, det_point, L, detect_time, detect_vel):

    posnew = np.zeros(N)
    velnew = np.zeros(N)
    den = 0
    flo = 0

    # Get new positions and velocities
    for i, car in enumerate(cars):
        posnew[i], velnew[i] = car.upd_pos_vel(params)

    # Second phase: After a certain number of steps, activate the detection loop
    if time_pass > time_measure:

        # Calculate global flow and density
        den, flo = flow_global(N, velnew, L)

        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if car.pos < det_point <= posnew[i]:

                # Linear interpolation to find the exact time and speed
                delta_t = (det_point - car.pos) / car.vel
                detect_time[i] = time_pass + delta_t
                detect_vel[i] = car.vel + car.acc * delta_t
    
    cars = vc.update_cars(cars, N, posnew, velnew, params, L)

    return cars, den, flo, detect_time, detect_vel

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
    params = [des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t, length]

    return Nmax, L, steps, steps_measure, splim, params, det_point

def init_simulation(N, L, params):
   
    cars = []
    length = params[7]
    min_gap = params[4]

    # Check if the road can accommodate all cars with the required spacing
    total_space_per_car = length + min_gap
    if N * total_space_per_car > L:
        raise ValueError(f"Cannot fit {N} cars on a road of length {L} with min_gap={min_gap} and car length={length}.")

    lane = 0
    vel = 0
    acc = 0
    pos = np.zeros(N)
    dv = 0

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (length + min_gap)

    # Calculate headway
    for i in range(N):
        next_car = (i + 1) % N
        headway = (pos[next_car] - pos[i] - length) % L

        car = vc(i, lane, pos[i], vel, acc, headway, dv)
        cars.append(car)

    print(N, 'Cars initialised')

    return cars

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

def Simulate_IDM(N, params, steps, steps_measure, det_point, L):

    del_t = params[6]
    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []
    detect_time = np.zeros(N)
    detect_vel = np.zeros(N)

    # Initialise cars
    cars = init_simulation(N, L, params)

    for i in range(steps):
        time_pass = i * del_t

        if time_pass > steps_measure * del_t:
            cars, den, flo, detect_time, detect_vel = Step(N, cars, params, time_pass, steps_measure * del_t, det_point, L, detect_time, detect_vel)

            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            cars, den, flo, _, _ = Step(N, cars, params, time_pass, steps_measure * del_t, det_point, L, [], [])

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel)

    print('Simulation for car total', N, 'completed')

    return glob_flow, glob_dens, loc_flow, loc_dens