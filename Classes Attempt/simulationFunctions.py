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
    desSpeed = 30 #np.random.uniform(20,33,N)
    pos = np.zeros(N)
    
    '''
    pos = np.linspace(0, L - 10, N, endpoint=False)

    # Ensure the minimum gap is maintained
    for i in range(0, N):
        i = i % N
        j = (i + 1) % N
 
        if pos[j] > pos[i]:
            if (pos[j] - pos[i]) < min_gap[i] + length[j]:
                pos[j] = (pos[i] + min_gap[i] + length[j]) % L
        elif pos[j] + L > pos[i]:
            if (pos[j] + L - pos[i]) < min_gap[i] + length[j]:
                pos[j] = (pos[i] + min_gap[i] + length[j]) % L
    
    '''
    # Calculate initial positions with min_gap
    for i in range(N):
        pos[i] = i * (length[i] + min_gap[i])
    #'''

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
    flow = np.mean(velnew) * dens * (3600 / 1000)

    return dens, flow



def Step(N, cars, time_pass, time_measure, det_point, L, detect_time, detect_vel, time_step):

    velnew = np.zeros(N)
    posnew = np.zeros(N)
    den = 0
    flo = 0 

    # Update positions and velocities
    cars, velnew, posnew = vc.upd_pos_vel(cars, time_step)
    
    # Detection and measurement logic (only for real cars)
    if time_pass > time_measure:
        den, flo = flow_global(N, velnew, L)

        # Detection loop for local measurements
        for i, car in enumerate(cars):
            if (car.pos[-1] < det_point <= posnew[i]):# or (car.pos[-1] < det_point <= posnew[i] + L):

                s = det_point - car.pos[-1]

                # Check if acceleration is significant
                if abs(car.acc[-1]) > 1e-6:
                    sqrt_term = car.vel[-1]**2 + 2 * car.acc[-1] * s
                    
                    # Ensure the sqrt term is non-negative
                    if sqrt_term >= 0:
                        delta_t = (-car.vel[-1] + np.sqrt(sqrt_term)) / car.acc[-1]
                    else:
                        delta_t = 0
                else:
                    delta_t = s / car.vel[-1] if car.vel[-1] > 0 else 0 
                
                # Store detection time and velocity at the exact moment of crossing det_point
                detect_time[i] = time_pass + delta_t
                detect_vel[i] = car.vel[-1] + car.acc[-1] * delta_t

    # Update variables
    cars = vc.update_cars(cars, N, L, posnew, velnew)

    return cars, den, flo, detect_time, detect_vel



def analyse_global(track_flow, track_dens):
    
    # Calculate overall global flow and density
    glob_flow = np.mean(track_flow)
    glob_dens = np.mean(track_dens)

    return glob_flow, glob_dens


#An alternative extension is the analysis of the model in terms of local densities and local flows. Measurements of flow and density 
# normally take place locally. At a detection point, the time at which a vehicle passes the detection point is measured, as well as 
# its velocity. 
# 
# Traffic flow is then measured by the number of cars that pass the point within a given time frame. The density is 
# then deduced by dividing the traffic flow by the average velocity of the traffic. Such a measurement gives rise to a local density
#  and a local flow. One possible follow-on question is thus how the local flow and local density compare to the global parameters
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
    loc_flow, loc_dens = analyse_local(track_det_time, track_det_vel, steps * time_step)

    print('Simulation for car total', N, 'completed')

    return cars, glob_flow, glob_dens, loc_flow, loc_dens