import numpy as np

class VehicleClass:

    def __init__(self, car_id, lane, pos, vel, acc, headway, dv, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

        self.car_id = car_id

        self.lane = lane
        self.pos = pos

        self.vel = vel
        self.acc = acc

        self.headway = headway
        self.dv = dv
        
        self.speedlim = speedlim
        self.des_speed = speedlim 
        self.des_speed_inv = 1.0 / speedlim if speedlim > 0 else 0

        self.acc_exp = acc_exp
        self.time_gap = time_gap
        self.min_gap = min_gap

        self.comf_decel = comf_decel
        self.acc_max = acc_max
        self.length = length

        self.comf_stopping_distance = -self.vel[-1] ** 2 / (-2 * self.comf_decel) if self.comf_decel > 0 else 0
        self.max_comf_stopping_distance = -self.des_speed_inv ** 2 / (-2 * self.comf_decel) if self.comf_decel > 0 else 0

    def upd_pos_vel(cars, time_step, L, traffic_light):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))

        light_status = traffic_light.status()

        for i, car in enumerate(cars):
            if car.car_id != -1:

                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

                # Calculate acceleration using IDM
                acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)

        for i, car in enumerate(cars):
            if car.car_id != -1:

                # Update velocity and position
                velnew[i] = car.vel[-1] + acc_new[i] * time_step
                posnew[i] = car.pos[-1] + car.vel[-1] * time_step + 0.5 * acc_new[i] * time_step**2

                # Ensure velocity does not go negative
                if velnew[i] < 0:

                    # Calculate time to stop
                    if abs(acc_new[i]) > 1e-6:
                        t_stop = -car.vel[-1] / acc_new[i]
                    else:
                        t_stop = 0

                    # Update position and velocity to stop at t_stop
                    posnew[i] = car.pos[-1] + car.vel[-1] * t_stop + 0.5 * acc_new[i] * t_stop**2
                    velnew[i] = 0
                
                car.stopping_distance = - car.vel[-1]**2 / (-2 * car.comf_decel)
        '''
        for i, car in enumerate(cars): 
            if car.car_id != -1:
                
                if light_status == 'orange':
                    
                    time_till_red = traffic_light.orange_duration - traffic_light.time_in_state
                    dist_can_travel = velnew[i] * (time_till_red - 0.5)

                    dist_to_light = (traffic_light.position - car.pos[-1]) % L

                    # Handle cases where the car can pass the light before red
                    if car.pos[-1] < traffic_light.position <= posnew[i]:

                        # The car is already passing the light
                        continue  
                    elif dist_to_light < dist_can_travel:
                        
                        # The car can safely pass before the light turns red
                        continue 
                    
                    # If stopping is required, adjust speed over the remaining time
                    elif dist_to_light >= dist_can_travel:

                        if (posnew[i] + dist_can_travel) % L >= (traffic_light.position - car.comf_stopping_distance) % L:

                            # Slow down gradually over time_till_red seconds
                            steps_remaining = int(time_till_red / 0.5)
                            if steps_remaining > 0:
                                acc_new[i] = -car.vel[-1] / time_till_red
                                velnew[i] = car.vel[-1] + acc_new[i] * time_till_red / steps_remaining
                                posnew[i] = car.pos[-1] + (velnew[i]**2 - car.vel[-1]**2) / (2 * acc_new[i]) - car.min_gap / steps_remaining
                            else:
                                velnew[i] = 0
                                posnew[i] = traffic_light.position - car.min_gap
                                acc_new[i] = -car.vel[-1]**2 / (2 * dist_to_light)

                    # If the car is within a stopping distance, ensure a full stop
                    elif dist_to_light <= car.comf_stopping_distance:
                            velnew[i] = 0
                            posnew[i] = traffic_light.position - car.min_gap
                            distance = (traffic_light.position - (car.pos[-1] - car.min_gap) % L) % L

                            if distance > 0:
                                acc_new[i] = -car.vel[-1]**2 / (2 * distance)
                            else:
                                acc_new[i] = 0
                    else:
                        index = (i + 1) % len(cars)
                        next_car = cars[index]

                        if next_car.car_id == -1:
                            index = (i + 2) % len(cars)
                            next_car = cars[index]

                        if next_car.vel[-1] == 0 and ((car.pos[-1] - car.length) % L - next_car.pos[-1]) % L < car.comf_stopping_distance:

                            safe_gap = car.min_gap + next_car.length
                            min_pos = (next_car.pos[-1] - safe_gap) % L

                            if car.pos[-1] < min_pos <= posnew[i]:
                                velnew[i] = 0
                                posnew[i] = min_pos
                                acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)
                    
                elif light_status == 'red':
                    
                    if car.car_id == 0 and car.pos[-1] > 90:
                        ...
                    # Handle stopping at red light
                    if car.pos[-1] == traffic_light.position - car.min_gap:
                        posnew[i] = car.pos[-1]
                        velnew[i] = 0
                        acc_new[i] = 0

                    # Handle coming up to traffic light
                    elif car.pos[-1] < traffic_light.position - car.min_gap <= posnew[i]:
                        
                        index = (i + 1) % len(cars)
                        next_car = cars[index]

                        if next_car.car_id == -1:
                            index = (i + 2) % len(cars)
                            next_car = cars[index]

                        if ((car.pos[-1] - car.length) % L - next_car.pos[-1]) % L < car.max_comf_stopping_distance:
                            safe_gap = car.min_gap + next_car.length
                            min_pos = (next_car.pos[-1] - safe_gap) % L

                            velnew[i] = 0
                            posnew[i] = min_pos                                
                            acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)

                        elif next_car.vel[-1] == 0 or velnew[index] == 0:

                            safe_gap = car.min_gap + next_car.length
                            min_pos = (next_car.pos[-1] - safe_gap) % L

                            if car.pos[-1] < min_pos <= posnew[i] or car.pos[-1] >= min_pos > posnew[i]:
                                velnew[i] = 0
                                posnew[i] = min_pos
                                acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)
                        else:
                            velnew[i] = 0
                            posnew[i] = (traffic_light.position - car.min_gap) % L
                            distance = (traffic_light.position - car.pos[-1]) % L
                            acc_new[i] = -car.vel[-1]**2 / (2 * distance)
                    else:
                        index = (i + 1) % len(cars)
                        next_car = cars[index]

                        if next_car.car_id == -1:
                            index = (i + 2) % len(cars)
                            next_car = cars[index]

                        if ((car.pos[-1] - car.length) % L - next_car.pos[-1]) % L < car.max_comf_stopping_distance:
                            safe_gap = car.min_gap + next_car.length
                            min_pos = (next_car.pos[-1] - safe_gap) % L

                            velnew[i] = 0
                            posnew[i] = min_pos                                
                            acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)

                        elif next_car.vel[-1] == 0 or velnew[index] == 0:

                            safe_gap = car.min_gap + next_car.length
                            min_pos = (next_car.pos[-1] - safe_gap) % L

                            if car.pos[-1] < min_pos <= posnew[i] or car.pos[-1] >= min_pos > posnew[i]:
                                velnew[i] = 0
                                posnew[i] = min_pos
                                acc_new[i] = -car.vel[-1]**2 / (2 * (min_pos - car.pos[-1]) % L)
                                
                # Ensure velocity does not go negative
                if velnew[i] < 0:
                    velnew[i] = 0
                                
        
                elif light_status == "orange":

                    # Calculate remaining time in the orange phase
                    cycle_time = traffic_light.green_duration + traffic_light.orange_duration + traffic_light.red_duration
                    time_in_cycle = current_time % cycle_time
                    time_left_in_orange = traffic_light.orange_duration - time_in_cycle if time_in_cycle < traffic_light.green_duration + traffic_light.orange_duration else 0
                    
                    distance_to_light = traffic_light.position - car.pos[-1]
                    # Calculate time required to reach the light at the current velocity
                    if velnew[i] > 0:
                        time_to_light = distance_to_light / velnew[i]
                    else:
                        time_to_light = float('inf')

                    # If the car can reach the light before it turns red, continue
                    if time_to_light > time_left_in_orange:
    
                        # Slow down to rest if it cannot make it in time
                        velnew[i] = 0
                        posnew[i] = car.pos[-1] + velnew[i] * time_step
                        acc_new[i] = -car.comf_decel
                
                
                elif light_status == 'orange':
                # Check if the car has enough time to pass through the light before it turns red
                time_to_light = dist_to_light / car.vel[-1] if car.vel[-1] > 0 else float('inf')
                if time_to_light >= time_step:

                    # The car needs to stop, so check if it can stop in time
                    if dist_to_light < stopping_distance:
                        # If the car is too close, apply hard deceleration to stop before hitting the light
                        acc_new[i] = -car.vel[-1]**2 / (2 * dist_to_light)
                        velnew[i] = 0  
                    else:
                        # If there's enough distance, apply comfortable deceleration to stop
                        acc_new[i] = -car.vel[-1]**2 / (2 * dist_to_light)
                        velnew[i] = 0
                '''

        # Set new position and velocity values
        for i, car in enumerate(cars):
            
            if car.car_id != -1:
                car.acc.append(acc_new[i])
                car.pos.append(posnew[i] % L)
                car.vel.append(velnew[i])

        return cars


    def update_cars(cars, N, L, time_step):
        acc = [car.acc[-1] for car in cars]

        for  i, car in enumerate(cars): 

            if car.car_id != -1:
                next_car = cars[(i + 1) % len(cars)]
                
                # Compute headway
                car.headway.append(((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L)

                if car.headway[-1] < car.min_gap and next_car.car_id != -1:

                    # Calculate desired bumper-to-bumper distance (s*)
                    s_star = car.min_gap

                    # Calculate acceleration using IDM
                    acc_new = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)
                    
                    # Update velocity and position
                    velnew = car.vel[-1] + acc_new * time_step

                    # Ensure velocity does not go negative
                    if velnew <= 0:
                        velnew = 0
                    
                    acc[i] = acc_new
                    car.vel[-1] = velnew

                car.acc.append(acc[i])
                car.dv.append(car.vel[-1] - next_car.vel[-1])

        return cars