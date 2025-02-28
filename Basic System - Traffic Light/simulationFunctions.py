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

            # Ensure minimum gap is maintained (remove this?)
            if headway < min_gap:

                changes = True

                # Move the car back to maintain the minimum gap
                pos[i] = ((pos[j] - length) % L - min_gap) % L

        if not changes:
            break

    # Create cars
    for i in range(N):

        next_car = (i + 1) % N
        headway = ((pos[next_car] - length) - pos[i]) % L

        car = vc(i, pos[i], initial_vel, initial_acc, headway, dv, speed_limit, acc_exp, time_gap, min_gap, comf_decel, acc_max, length)
        cars.append(car)

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
    
    # Create phantom car at traffic light position
    phantom_car = vc(-1, light_pos, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10)

    cars.sort(key=lambda car: car.pos[-1])
    
    for i in reversed(range(len(cars))):

        car = cars[i]

        # Check if car can make it through
        s = (phantom_car.pos[-1] - car.pos[-1]) % L

        max_s = car.vel * (traffic_light.orange_duration - traffic_light.time_in_state)

        if max_s <= s and car.pos[-1] <= phantom_car.pos[-1]:

            cars.insert(i + 1, phantom_car)
            break

    return cars

def remove_phantom_car(cars, L):
    
    new_cars = []

    for i, car in enumerate(cars):

        next_car = cars[(i + 1) % len(cars)]
        
        if next_car.car_id == -1:
            next_car = cars[(i + 2) % len(cars)]
            car.headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L
            car.dv = car.vel - next_car.vel

        if car.car_id != -1:
            new_cars.append(car)
        else:
            del(car)
    
    new_cars.sort(key=lambda car: car.pos[-1])

    return new_cars

def get_headway(cars, L):

    for i in range(len(cars)):

        # Check if the current car is a phantom car
        if cars[i].car_id == -1:

            # Get the next car and previous car
            car = cars[i]
            next_car = cars[(i + 1) % len(cars)]
            prev_car = cars[(i - 1) % len(cars)]

            # Calculate headway between previous car and next car and traffic light
            headway_prev = ((car.pos[-1] - car.length) % L - prev_car.pos[-1]) % L
            headway_next = ((next_car.pos[-1] - next_car.length) % L - prev_car.pos[-1]) % L

            # Set to min headway
            prev_car.headway = min(headway_prev, headway_next)

            # Check if dv is needed to be updated
            if prev_car.headway == headway_next:
                prev_car.dv = prev_car.dv - next_car.dv
            break
    
    return cars

def Step(N, cars, time_pass, time_measure, det_point, L, time_step, traffic_light):

    # Measurement variables
    den = 0
    flo = 0
    detect_time = []
    detect_vel = []
    time_left = traffic_light.orange_duration - traffic_light.time_in_state

    # Update traffic light state
    traffic_light.update(time_step)
    light_state = traffic_light.status()
    
    # Handle state transitions
    if light_state == 'red':

        # Find the index to insert the phantom car without sorting
        insert_index = None
        for i in range(len(cars)):
            if cars[i].pos[-1] >= traffic_light.position:
                insert_index = i
                break

        # Create the phantom car
        phantom_car = vc(-1, [traffic_light.position], [0], [0], [0], [0], 0, 0, 0, 0, 0, 0, 0)
        
        # Insert the phantom car at the correct position
        if insert_index is not None:
            cars.insert(insert_index, phantom_car)

            cars[i - 1].headway = ((cars[i].pos[-1] - cars[i - 1].pos[-1]) % L)
            cars[i - 1].dv = cars[i - 1].vel
        else:
            cars.append(phantom_car)

            cars[-2].headway = ((cars[-1].pos[-1] - cars[-2].pos[-1]) % L)
            cars[-2].dv = cars[-2].vel
            
    elif light_state == 'orange' and time_left > 4:
        cars = remove_phantom_car(cars, L)
        cars = add_phantom_car(cars, traffic_light, L)
    elif light_state == 'green' and len(cars) > N:
        cars = remove_phantom_car(cars, L)

    if light_state == 'green':
        ...#cars.sort(key=lambda car: car.pos[-1])

    if light_state == 'orange':
        ...#cars = get_headway(cars, L)

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

def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L, green_duration):

    # Declare variables
    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = [] 

    traffic_light = tl(L/2, green_duration, 0, 100 - green_duration)

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

    print('Simulation for green duration =', green_duration, 'completed')

    cars = remove_phantom_car(cars, L)

    return cars, glob_flow, glob_dens, loc_flow, loc_dens