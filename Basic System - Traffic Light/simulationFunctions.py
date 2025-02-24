import numpy as np
from VehicleClass import VehicleClass as vc
from TrafficLightClass import TrafficLightClass as tl

def init_simulation(N, L,):

    cars = []
    initial_vel = 0
    initial_acc = 0
    dv = 0
    length = 5
    min_gap = 2
    acc_exp = 4
    time_gap = 1
    comf_decel = 1.5
    acc_max = 1
    speed_limit = 70 / 2.237

    # Set initial positions
    pos = np.zeros(N)
    for i in range(N):
        pos[i] = i * (length + min_gap)

    # Ensure the minimum gap is maintained
    while True:

        changes = False

        # Loop to adjust positions
        for i in range(N):

            # Next car index
            j = (i + 1) % N

            # Compute headway
            headway = ((pos[j] - length) % L - pos[i]) % L

            # Ensure minimum gap is maintained
            if headway < min_gap:

                changes = True

                # Move the car back to maintain the minimum gap
                pos[i] = ((pos[j] - length) % L - min_gap) % L

        if not changes:
            break

    # Create cars
    for i in range(N - 1):

        next_car = (i + 1) % N
        headway = ((pos[next_car] - length) - pos[i]) % L

        car = vc(i, pos[i], initial_vel, initial_acc, headway, dv, speed_limit, acc_exp, time_gap, min_gap, comf_decel, acc_max, length)
        cars.append(car)

    print(N, 'Cars initialised')
    return cars

def flow_global(N, velocities, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate avergae velocity
    avg_vel = np.mean(velocities)

    # Calculate global flow (cars per hour)
    flow = dens * avg_vel * 3.6

    return dens, flow

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

def add_phantom_car(cars, traffic_light, L):

    # Get the traffic light position
    light_pos = traffic_light.position
    
    # Find if there are cars before and after the traffic light
    cars_before = [car for car in cars if car.pos[-1] < light_pos]
    cars_after = [car for car in cars if car.pos[-1] >= light_pos]
    
    # Create phantom car at traffic light position
    phantom_car = vc(-1, light_pos, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    for i in range(len(cars_before ) - 1, -1, -1):

        car = cars_before[i]

        # Check if car can make it through
        s = (light_pos - car.pos[-1])

        max_s = car.vel * traffic_light.orange_duration

        if s < max_s:
            continue
        else:

            cars_before.insert(i + 1, phantom_car)
            if i + 2 < len(cars_before):
                cars_before[i + 2].headway = (cars_before[i + 2].pos[-1] - light_pos) % L
            break
    
    # Combine lists
    new_cars = cars_before + cars_after

    return new_cars

def remove_phantom_car(cars, L):
    
    # Remove the phantom car
    cars = [car for car in cars if car.car_id != -1]

    for i, car in enumerate(cars):
        next_car = cars[(i + 1) % len(cars)]

        car.headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L
        car.dv = car.vel - next_car.vel
        
    return cars

def Step(N, cars, time_pass, time_measure, det_point, L, time_step, traffic_light):

    # Measurement variables
    den = 0
    flo = 0
    detect_time = []
    detect_vel = []

    # Update traffic light state
    traffic_light.update(time_step)
    
    light_state = traffic_light.status()

    if light_state == 'orange':
        cars = add_phantom_car(cars, traffic_light, L)
    elif light_state == 'green':
        cars = remove_phantom_car(cars, L)

    '''
    # Store previous velocity and acceleration
    for car in cars:
        car.prev_vel = car.vel
        car.prev_acc = car.acc

    # Update headway and dv
    for i in range(N):
        next_car = cars[(i + 1) % N]
        cars[i].headway = ((next_car.pos[-1] - next_car.length) % L - cars[i].pos[-1]) % L
        cars[i].dv = cars[i].vel - next_car.vel

    # Calculate new accelerations
    acc_new = np.zeros(N)
    for i, car in enumerate(cars):
        s_star = car.min_gap + max(0, car.vel * car.time_gap + (car.vel * car.dv) / (2 * np.sqrt(car.acc_max * car.comf_decel)))
        acc_new[i] = car.acc_max * (1 - (car.vel / car.des_speed)**car.acc_exp - (s_star / car.headway)**2)

    # Update velocities and positions
    for i, car in enumerate(cars):

        vel_new = car.vel + acc_new[i] * time_step
        t = time_step
        if vel_new < 0:
            t  = - car.vel / acc_new[i]
            vel_new = car.vel + acc_new[i] * t

        pos_new = (car.pos[-1] + car.vel * t + 0.5 * acc_new[i] * t**2) % L
        car.acc = acc_new[i]
        car.vel = vel_new
        car.pos.append(pos_new)

    # Prevent overtaking or crashing
    for i in range(N):
        next_car = cars[(i + 1) % N]
        headway = ((next_car.pos[-1] - next_car.length) % L - cars[i].pos[-1]) % L
        if headway < car.min_gap / 2:
            cars[i].pos[-1] = (next_car.pos[-1] - next_car.length - cars[i].min_gap) % L
    '''

    cars = vc.update_cars(cars, time_step, L)

    # Take measurements
    if time_pass > time_measure:
        den, flo = flow_global(N, [car.vel for car in cars if car.car_id != -1], L)

        # Detection loop for local measurements
        for car in cars:
            if car.car_id != -1:
                if (car.pos[-2] < det_point <= car.pos[-1]) or (car.pos[-1] < car.pos[-2] and (car.pos[-2] < det_point <= car.pos[-1] + L)):

                    # Calculate distance from initial position to detection point
                    s = det_point - car.pos[-2]

                    # Calculate delta t
                    if car.prev_acc == 0:
                        delta_t = s / car.prev_vel if car.prev_vel != 0 else 0
                    else:
                        sqrt_term = car.prev_vel**2 + 2 * car.prev_acc * s
                        delta_t = (-car.prev_vel + np.sqrt(sqrt_term)) / car.prev_acc

                    # Store detection time and velocity
                    detect_time.append(time_pass - time_step + delta_t)
                    detect_vel.append(car.prev_vel + car.prev_acc * delta_t)

    return cars, den, flo, detect_time, detect_vel

def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L):

    # Declare variables
    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []
    traffic_light = tl(150, 10, 5, 10)

    # Initialise cars
    cars = init_simulation(N, L)

    for i in range(steps):

        time_pass = i * time_step

        cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step, traffic_light)

        if time_pass > steps_measure * time_step:
            track_flow.append(flo)
            track_dens.append(den)
            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel, steps * time_step)

    print('Simulation for car total', N, 'completed')

    return cars, glob_flow, glob_dens, loc_flow, loc_dens