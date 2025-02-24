import numpy as np
from VehicleClass import VehicleClass as vc

def init_simulation(N, L):
    np.random.seed(20)
    cars = []
    lane = 0
    vel = [0] * N
    acc = [0] * N
    pos = np.zeros(N)
    dv = [0] * N
    length = [5] *  N
    min_gap = [2] * N
    accexp = 4
    desSpeed = 70 / 2.3

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = (i) * (length[i] + min_gap[i])

    # Ensure the minimum gap is maintained
    while True:

        changes = False

        # Loop to adjust positions
        for i in range(N):
            
            # Next car index
            j = (i + 1) % N 

            min_safe_gap = min_gap[i]

            # Compute headway
            headway = ((pos[j] - length[j]) % L - pos[i]) % L
            
            # Ensure minimum gap is maintained
            if headway < min_safe_gap:
                
                changes = True

                # Move the car back to maintain the minimum gap
                pos[i] = (pos[j] - length[j] - min_safe_gap) % L
        
        if not changes:
            break

    # Calculate headway
    for i in range(N-1):
        next_car = (i + 1) % N
        headway = [(pos[next_car] - pos[i] - length[next_car]) % L]

        car = vc(i, lane, [pos[i]], [vel[i]], [acc[i]], headway, [dv[i]], desSpeed, accexp, 1, min_gap[i], 1.5, 1, length[i])
        cars.append(car)
    
    car = vc(i, lane, [pos[i]], [0], [0], headway, [dv[i]], 0, 0, 1, min_gap[i], 1.5, 0, length[i])
    cars.append(car)

    print(N, 'Cars initialised')

    return cars



def flow_global(N, velnew, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.mean(velnew) * dens * 3.6

    return dens, flow



def Step(N, cars, time_pass, time_measure, det_point, L, time_step):

    den = 0
    flo = 0
    detect_time = []
    detect_vel = []

    # Update cars
    cars = vc.update_cars(cars, time_step, L)
    
    # Detection and measurement logic (only for real cars)
    if time_pass > time_measure:
        den, flo = flow_global(N, [car.vel[-1] for car in cars], L)

        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if (car.pos[-2] < det_point <= car.pos[-1]) or (car.pos[-1] < car.pos[-2] and car.pos[-2] < det_point <= car.pos[-1] + L):

                s = det_point - car.pos[-2]

                # Calculate delta t
                if car.acc[-2] == 0:
                    delta_t = s / car.vel[-2]
                else:
                    sqrt_term = car.vel[-2]**2 + 2 * car.acc[-2] * s
                    delta_t = (-car.vel[-2] + np.sqrt(sqrt_term)) / car.acc[-2]
                    
                # Store detection time and velocity
                detect_time.append(time_pass + delta_t)
                detect_vel.append(car.vel[-2] + car.acc[-2] * delta_t)
    
    return cars, den, flo, detect_time, detect_vel



def analyse_global(track_flow, track_dens):
    
    # Calculate overall global flow and density
    glob_flow = np.mean(track_flow)
    glob_dens = np.mean(track_dens)

    return glob_flow, glob_dens



def analyse_local(track_det_time, track_det_vel, time):

    # Avoid division by zero if no detection data
    if not track_det_time or not track_det_vel:
        return 0, 0 
    
    num_cars = len(track_det_time)
    
    # Calculate local flow
    loc_flow = (num_cars / time) * 3600

    # Calculate local density
    avg_speed = sum(track_det_vel) / num_cars

    loc_dens = loc_flow / (avg_speed * 3.6)

    return loc_flow, loc_dens



def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L):

    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []

    # Initialise cars
    cars = init_simulation(N, L)

    for i in range(steps):

        time_pass = i * time_step

        if time_pass > steps_measure * time_step:
            cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step)

            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            cars, den, flo, _, _ = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel, steps * time_step)

    print('Simulation for car total', N, 'completed')

    return cars, glob_flow, glob_dens, loc_flow, loc_dens