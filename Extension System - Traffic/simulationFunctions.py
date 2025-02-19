import numpy as np
from VehicleClass import VehicleClass as vc

def init_simulation(N, L):
    np.random.seed(20)
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
    pos = np.zeros(N)

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (length[i] + min_gap[i])

    # Ensure the minimum gap is maintained
    while True:

        changes = False

        # Loop to adjust positions
        for i in range(N):
            
            # Next car index
            j = (i + 1) % N 

            min_safe_gap = min_gap[i]

            # Compute headway
            if pos[j] - length[j] >= pos[i]:
                headway = pos[j] - pos[i] - length[j]
               
            else:
                headway = (pos[j] - length[j] + L - pos[i])
                
            # Ensure minimum gap is maintained
            if headway < min_safe_gap:
                
                changes = True

                # Move the car back to maintain the minimum gap
                pos[i] = (pos[j] - min_safe_gap) % L
        
        if not changes:
            break

    # Calculate headway
    for i in range(N):
        next_car = (i + 1) % N
        headway = [(pos[next_car] - pos[i] - length[next_car]) % L]

        car = vc(i, lane, [pos[i]], vel, acc, headway, dv, desSpeed, accexp, 1, min_gap[i], 1.5, 1, length[i])
        cars.append(car)

    print(N, 'Cars initialised')

    return cars



def flow_global(N, velnew, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.mean(velnew) * dens * (3600 / 1000)

    return dens, flow

def add_phantom_car(cars, traffic_light, L):

    # Find the index to insert the phantom car without sorting
    insert_index = None
    for i in range(len(cars)):
        if cars[i].pos[-1] >= traffic_light.position:
            insert_index = i
            break

    # Create the phantom car
    phantom_car = vc(-1, 0, [traffic_light.position], cars[i-1].vel, [0], [0], [0], 0, 0, 0, 0, 0, 0, 0)
    
    # Insert the phantom car at the correct position
    if insert_index is not None:
        cars.insert(insert_index, phantom_car)

        cars[i - 1].headway[-1] = ((cars[i].pos[-1] - cars[i - 1].pos[-1]) % L)
        cars[i - 1].dv[-1] = cars[i - 1].vel[-1]
    else:
        cars.append(phantom_car)

        cars[-2].headway[-1] = ((cars[-1].pos[-1] - cars[-2].pos[-1]) % L)
        cars[-2].dv[-1] = cars[-2].vel[-1]

    return cars

def remove_phantom_car(cars):
    
    # Remove the phantom car
    cars = [car for car in cars if car.car_id != -1]

    return cars



def Step(N, cars, time_pass, time_measure, det_point, L, time_step, traffic_light):

    den = 0
    flo = 0 
    detect_time = []
    detect_vel = []

    # Get traffic light status
    light_status = traffic_light.status(time_pass)
    traffic_light.time_in_state += time_step

    #'''
    # Add phantom car based on traffic light status
    if light_status == "red" and len(cars) == N:
        cars = add_phantom_car(cars, traffic_light, L)
    
    # Remove phantom car based on traffic light status
    elif light_status == 'red' and len(cars) > N:
        cars = remove_phantom_car(cars)
    #'''

    # Update positions and velocities
    cars = vc.upd_pos_vel(cars, time_step, L, traffic_light, light_status, time_pass)

    # Update variables
    cars = vc.update_cars(cars, N, L, time_step)

    # Detection and measurement logic (only for real cars)
    if time_pass > time_measure:
        den, flo = flow_global(N, [car.vel[-1] for car in cars if car.car_id != -1], L)
        
        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if car.car_id != -1 and ((car.pos[-2] < det_point <= car.pos[-1]) or (car.pos[-1] < car.pos[-2] and car.pos[-2] < det_point <= car.pos[-1] + L)):

                s = (det_point - car.pos[-2]) % L
                
                # Calculate delta t
                if car.acc[-2] == 0:
                    delta_t = s / car.vel[-2]
                else:
                    sqrt_term = car.vel[-2]**2 + 2 * car.acc[-2] * s

                    if sqrt_term < 0:
                        print(car.car_id, sqrt_term, car.acc[-2], car.acc[-1], car.vel[-2], s)
                    delta_t = (-car.vel[-2] + np.sqrt(sqrt_term)) / car.acc[-2]
                    
                # Store detection time and velocity
                detect_time.append(time_pass + delta_t)
                detect_vel.append(car.vel[-2] + car.acc[-2] * delta_t)

    # Reset light time
    if (traffic_light.time_in_state == traffic_light.green_duration and light_status == 'green') or (traffic_light.time_in_state == traffic_light.orange_duration and light_status == 'orange') or (traffic_light.time_in_state == traffic_light.red_duration and light_status == 'red'):
        traffic_light.time_in_state = 0
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



def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L, traffic_light):

    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []

    # Initialise cars
    cars = init_simulation(N, L)

    for i in range(steps):
        
        time_pass = i * time_step

        if time_pass > steps_measure * time_step:
            cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step, traffic_light)

            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            cars, den, flo, _, _ = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step, traffic_light)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel, steps * time_step)

    print('Simulation for car total', N, 'completed')

    cars = remove_phantom_car(cars)

    return cars, glob_flow, glob_dens, loc_flow, loc_dens