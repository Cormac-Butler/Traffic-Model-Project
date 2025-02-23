import numpy as np
from VehicleClass import VehicleClass as vc

def init_simulation(N, L):
    np.random.seed(20)
    cars = []
    lane = 0
    initial_vel = 0
    length = 3
    min_gap = 2
    acc_exp = 4
    des_speed = 30
    time_gap = 1
    comf_decel = 1.5
    acc_max = 1
    speedlim = des_speed

    # Set initial positions
    pos = np.zeros(N)
    for i in range(N):
        pos[i] = i * (length + min_gap)

    # Adjust positions to maintain min_gap on road
    while True:
        changes = False
        for i in range(N):
            j = (i + 1) % N
            headway = (pos[j] - length - pos[i]) % L
            if headway < min_gap:
                changes = True
                pos[i] = (pos[j] - length - min_gap) % L
        if not changes:
            break

    # Create cars
    for i in range(N):
        car = vc(i, lane, pos[i], initial_vel, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length)
        cars.append(car)

    print(N, 'Cars initialised')
    return cars

def flow_global(N, velocities, L):

    dens = N / (L / 1000)
    avg_vel = np.mean(velocities)
    flow = dens * avg_vel * 3.6

    return dens, flow

def analyse_global(track_flow, track_dens):

    glob_flow = np.mean(track_flow)
    glob_dens = np.mean(track_dens)

    return glob_flow, glob_dens

def analyse_local(track_det_time, track_det_vel, time):

    if not track_det_time or not track_det_vel:
        return 0, 0
    
    num_cars = len(track_det_time)
    loc_flow = (num_cars / time) * 3600
    avg_speed = sum(track_det_vel) / num_cars
    loc_dens = loc_flow / (avg_speed * 3.6)

    return loc_flow, loc_dens

def Step(N, cars, time_pass, time_measure, det_point, L, time_step):

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
        if vel_new < 0:
            vel_new = 0

        pos_new = (car.pos[-1] + car.vel * time_step + 0.5 * acc_new[i] * time_step**2) % L
        car.acc = acc_new[i]
        car.vel = vel_new
        car.pos.append(pos_new)

    # Prevent overtaking or crashing
    for i in range(N):
        next_car = cars[(i + 1) % N]
        headway = ((next_car.pos[-1] - next_car.length) % L - cars[i].pos[-1]) % L
        if headway < 0:
            cars[i].vel = 0

            # Already was told not to edit position
            cars[i].pos[-1] = ((next_car.pos[-1] - next_car.length) % L - cars[i].min_gap) % L

    # Measurement variables
    den = 0
    flo = 0
    detect_time = []
    detect_vel = []

    # Take measurements
    if time_pass > time_measure:
        den, flo = flow_global(N, [car.vel for car in cars], L)

        for i, car in enumerate(cars):
            if (car.pos[-2] < det_point <= car.pos[-1]) or \
               (car.pos[-1] < car.pos[-2] and (car.pos[-2] < det_point or det_point <= car.pos[-1])):
                s = det_point - car.pos[-2]
                if car.prev_acc == 0:
                    delta_t = s / car.prev_vel if car.prev_vel != 0 else 0
                else:
                    sqrt_term = car.prev_vel**2 + 2 * car.prev_acc * s
                    delta_t = (-car.prev_vel + np.sqrt(sqrt_term)) / car.prev_acc if sqrt_term >= 0 else 0
                detect_time.append(time_pass - time_step + delta_t)
                detect_vel.append(car.prev_vel + car.prev_acc * delta_t)

    return cars, den, flo, detect_time, detect_vel

def Simulate_IDM(N, time_step, steps, steps_measure, det_point, L):

    track_flow = []
    track_dens = []
    track_det_time = []
    track_det_vel = []

    cars = init_simulation(N, L)

    for i in range(steps):
        time_pass = i * time_step
        cars, den, flo, detect_time, detect_vel = Step(N, cars, time_pass, steps_measure * time_step, det_point, L, time_step)

        if time_pass > steps_measure * time_step:
            track_flow.append(flo)
            track_dens.append(den)
            track_det_time.extend(detect_time)
            track_det_vel.extend(detect_vel)

    glob_flow, glob_dens = analyse_global(track_flow, track_dens)
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel, steps * time_step)

    print('Simulation for car total', N, 'completed')

    return cars, glob_flow, glob_dens, loc_flow, loc_dens