import numpy as np
from VehicleClass import VehicleClass as vc

def init_simulation(N, L):
   
    cars = []

    lane = 0
    vel = [0]
    acc = [0]
    pos = np.zeros(N)
    dv = [0]
    length = [3] *  N
    min_gap = [2] * N
    accexp = 4
    desSpeed = 30

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (length[i] + min_gap[i])

    # Calculate headway
    for i in range(N):
        next_car = (i + 1) % N
        headway = [length[next_car] + min_gap[i]]

        car = vc(i, lane, [pos[i]], vel, acc, headway, dv, desSpeed, accexp, 1, min_gap[i], 1.5, 1, length[i])
        cars.append(car)

    print(N, 'Cars initialised')

    return cars



def flow_global(N, velnew, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.sum(velnew) / L * 3600

    return dens, flow



def Step(N, cars, time_pass, time_measure, det_point, L, detect_time, detect_vel, time_step):

    posnew = np.zeros(N)
    velnew = np.zeros(N)
    den = 0
    flo = 0

    # Get new positions and velocities
    for i, car in enumerate(cars):
        car, velnew[i] = car.upd_pos_vel(L, time_step)

    # Second phase: After a certain number of steps, activate the detection loop
    if time_pass > time_measure:

        # Calculate global flow and density
        den, flo = flow_global(N, velnew, L)

        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if (car.pos[-2] < det_point <= posnew[-1]) or (car.pos[-2] < det_point <= posnew[-1] + L):

                s = det_point - car.pos[-2]

                # Check if acceleration is significant
                if abs(car.acc[-2]) > 1e-6:
                    sqrt_term = car.vel[-2]**2 + 2 * car.acc[-2] * s
                    
                    # Ensure the sqrt term is non-negative
                    if sqrt_term >= 0:
                        delta_t = (-car.vel[-2] + np.sqrt(sqrt_term)) / car.acc[-2]
                    else:
                        delta_t = 0
                else:
                    delta_t = s / car.vel[-2] if car.vel[-2] > 0 else 0 
                
                # Store detection time and velocity at the exact moment of crossing det_point
                detect_time[i] = time_pass + delta_t
                detect_vel[i] = car.vel[-2] + car.acc[-2] * delta_t#
                
    cars = vc.update_cars(cars, N, L)

    return cars, den, flo, detect_time, detect_vel



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



def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L):

    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []
    detect_time = np.zeros(N)
    detect_vel = np.zeros(N)

    # Initialise cars
    cars = init_simulation(N, L)

    for i in range(steps):
        time_pass = i * time_step

        if time_pass > steps_measure * time_step:
            cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, detect_time, detect_vel, time_step)

            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            cars, den, flo, _, _ = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, [], [], time_step)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel)

    print('Simulation for car total', N, 'completed')

    return glob_flow, glob_dens, loc_flow, loc_dens, track_flow, track_dens