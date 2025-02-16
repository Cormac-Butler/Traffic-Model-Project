import numpy as np

class VehicleClass:

    def __init__(self, car_id, lane, pos, vel, acc, headway, dv, 
                 speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

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

    def upd_pos_vel(cars, time_step, L, traffic_light, light_status):
        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))
        dist_to_light = 0

        # Find the position of the last car in the queue (if any)
        queue_position = None
        for car in cars:
            dist_to_light = (traffic_light.position - car.pos[-1]) % L
            if dist_to_light < 10 and car.vel[-1] == 0:  # Car is stopped within 10m of the light
                queue_position = car.pos[-1]
                break

        for i, car in enumerate(cars):

            if traffic_light.position < car.pos[-1]:
                dist_to_light = L - (car.pos[-1] - traffic_light.position)

            else:
                dist_to_light = L - (traffic_light.position - car.pos[-1])

            # Check if the car has already passed the traffic light
            if dist_to_light > 0: 

                # Car is past the light, so it should continue moving normally using IDM
                s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap)
                acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)
            else:

                # Car is approaching the traffic light
                if light_status == "red":

                    # Calculate the stopping distance
                    if car.vel[-1] > 0: 
                        stopping_distance = car.vel[-1]**2 / (2 * car.comf_decel)
                    else:
                        stopping_distance = 1e-6 

                    # Check if the car is within stopping distance or in a queue
                    if dist_to_light < stopping_distance or dist_to_light < 10 or (queue_position is not None and car.pos[-1] >= queue_position):
                        
                        # Gradually decelerate based on distance to the light
                        deceleration_factor = min(1, dist_to_light / stopping_distance)
                        acc_new[i] = -car.comf_decel * deceleration_factor
                    else:
                        
                        # Car is not in stopping distance, so it should continue driving normally
                        s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))
                        acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)

                elif light_status == "orange":
                    
                    # Gradually slow down if within stopping distance
                    if car.vel[-1] > 0:  # Only calculate stopping distance if the car is moving
                        stopping_distance = car.vel[-1]**2 / (2 * car.comf_decel)
                    else:
                        stopping_distance = 1e-6 

                    if dist_to_light < stopping_distance:
                        deceleration_factor = min(1, dist_to_light / stopping_distance)
                        acc_new[i] = -car.comf_decel * deceleration_factor
                    else:
                        
                        # Car is not in stopping distance, so it should continue driving normally
                        s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))
                        acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)

                elif light_status == "green":
                    
                    # Resume normal driving if the light is green
                    s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))
                    acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / car.headway[-1])**2)

        for i, car in enumerate(cars):
            
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

        # Set new position and velocity values
        for i, car in enumerate(cars):
            car.acc.append(acc_new[i])
            car.pos.append(posnew[i] % L)
            car.vel.append(velnew[i])

        return cars


    def update_cars(cars, N, L, time_step):

        while True:
            changes = False

            # Loop backward to adjust positions for safety gaps
            for i in range(N - 1, -1, -1):
                car = cars[i]
                next_car = cars[(i + 1) % N]
                displacement = 0 

                # Get minimum safety gap
                min_safe_gap = car.min_gap + next_car.length

                # Compute headway
                if next_car.pos[-1] > car.pos[-1]:
                    car.headway.append(next_car.pos[-1] - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] + L - car.pos[-1])

                if car.headway[-1] < min_safe_gap:
                    
                    changes = True
                    car.headway[-1] = min_safe_gap

                    # Adjust the position of the current car to maintain the minimum gap
                    car.pos[-1] = (next_car.pos[-1] - min_safe_gap) % L

                    displacement = car.pos[-1] - car.pos[-2]

                    if displacement < 0:
                        displacement += L

                    car.acc[-1] = 2 * (displacement - car.vel[-2] * time_step) / time_step**2
                    car.vel[-1] = 2 * displacement / time_step - car.vel[-2]
            
            if not changes:
                break

        for  i, car in enumerate(cars):

            next_car = cars[(i + 1) % N]

            car.dv.append(next_car.vel[-1] - car.vel[-1])
        
        return cars