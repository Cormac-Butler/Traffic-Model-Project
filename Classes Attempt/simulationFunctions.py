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

    '''
    # Generate random positions for cars while maintaining the minimum gap
    pos = np.sort(np.random.uniform(0, L, N))

    # Ensure the minimum gap is maintained
    for i in range(1, N):
        if pos[i] - pos[i - 1] < min_gap[i - 1] + length[i]:
            pos[i] = pos[i - 1] + min_gap[i - 1] + length[i]

    # Wrap around the last car to ensure it doesn't overlap with the first car
    if (pos[-1] + min_gap[-1] + length[0]) % L < pos[0]:
        pos[-1] = (pos[0] - min_gap[-1] - length[0]) % L
    '''

    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (length[i] + min_gap[i])

    # Calculate headway
    for i in range(N):
        next_car = (i + 1) % N
        headway = [(pos[next_car] - pos[i]) % L]

        car = vc(i, lane, [pos[i]], vel, acc, headway, dv, desSpeed, accexp, 1, min_gap[i], 1.5, 1, length[i])
        cars.append(car)

    print(N, 'Cars initialised')

    return cars



def flow_global(N, velnew, L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)

    # Calculate global flow (cars per hour)
    flow = np.mean(velnew) * dens * 3600 / 1000

    return dens, flow



def Step(N, cars, time_pass, time_measure, det_point, L, detect_time, detect_vel, time_step, speed_limit_zones, traffic_light):

    '''
    # Update the traffic light state
    traffic_light.update(time_step)

    # Get the phantom car (if the light is red)
    phantom_car = traffic_light.get_phantom_car()

    # Add the phantom car to the list of cars (if it exists)
    if phantom_car is not None:
        cars_with_phantom = cars + [phantom_car]
        N += 1 
    else:
        cars_with_phantom = cars
    '''
    velnew = np.zeros(N)
    den = 0
    flo = 0

    # Update positions and velocities for all cars (including the phantom car)
    for i, car in enumerate(cars):
        car, velnew[i] = car.upd_pos_vel(L, time_step, speed_limit_zones, traffic_light)

    # Update headway and velocity differences using the cars_with_phantom list
    cars = vc.update_cars(cars, N, L)

    '''
    # Remove the phantom car from the list before further processing
    if phantom_car is not None:
        cars_with_phantom = cars_with_phantom[:-1]
    '''

    # Detection and measurement logic (only for real cars)
    if time_pass > time_measure:
        den, flo = flow_global(N, velnew, L)

        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if (car.pos[-2] < det_point <= car.pos[-1]) or (car.pos[-2] < det_point <= car.pos[-1] + L):

                s = det_point - car.pos[-2]

                # Check if acceleration is significant
                if abs(car.acc[-1]) > 1e-6:
                    sqrt_term = car.vel[-2]**2 + 2 * car.acc[-1] * s
                    
                    # Ensure the sqrt term is non-negative
                    if sqrt_term >= 0:
                        delta_t = (-car.vel[-2] + np.sqrt(sqrt_term)) / car.acc[-1]
                    else:
                        delta_t = 0
                else:
                    delta_t = s / car.vel[-2] if car.vel[-2] > 0 else 0 
                
                # Store detection time and velocity at the exact moment of crossing det_point
                detect_time[i] = time_pass + delta_t
                detect_vel[i] = car.vel[-2] + car.acc[-1] * delta_t
                
    #cars = vc.update_cars(cars, N, L)

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
    
    # Define time window for local measurements
    time_window = max(track_det_time) - min(track_det_time)

    if time_window == 0:
        return 0, 0  

    # Local flow: Number of cars passing the detection point per unit time (cars/hour)
    loc_flow = len(track_det_time) / time_window * 3600  

    # Local density: Using the fundamental equation (cars/km)
    avg_velocity = np.mean(track_det_vel)
    if avg_velocity == 0:
        return loc_flow, 0 

    loc_dens = loc_flow / (avg_velocity * 3.6)

    return loc_flow, loc_dens


def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L, speed_limit_zones, traffic_light):

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
            cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, detect_time, detect_vel, time_step, speed_limit_zones, traffic_light)

            track_flow.append(flo)
            track_dens.append(den)

            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)
        else:
            cars, den, flo, _, _ = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, [], [], time_step, speed_limit_zones, traffic_light)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel)

    print('Simulation for car total', N, 'completed')

    return glob_flow, glob_dens, loc_flow, loc_dens