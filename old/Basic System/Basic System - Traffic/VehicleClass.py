import numpy as np
from TrafficLightClass import TrafficLightClass as tl

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
    
    def update_cars(cars, time_step, L, traffic_light):

        # Update acc based on IDM
        acc_new = VehicleClass.calc_acc(cars, traffic_light, L)

        # Update velocity and position
        pos_new, vel_new = VehicleClass.upd_pos_vel(cars, time_step, acc_new)
        
        for i, car in enumerate(cars):

            # Set new position and velocity values
            car.acc.append(acc_new[i])
            car.pos.append(pos_new[i] % L)
            car.vel.append(vel_new[i])

            car.comf_stopping_distance = - car.vel[-1]**2 / (-2 * car.comf_decel) if car.car_id != -1 else 0
        
        cars = VehicleClass.update_headway_dv(cars, L, time_step)
            
        return cars
    
    def calc_acc(cars, traffic_light, L):

        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):
            if car.car_id!= -1:

                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

                # Calculate acceleration using IDM
                acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)

                # Check if car needs to respond to traffic light
                distance_to_light = (traffic_light.position - car.pos[-1]) % L
                
                # Only consider cars that haven't passed the light yet and are within reasonable distance
                safe_reaction_distance = car.comf_stopping_distance + 50
                
                if 0 < distance_to_light < safe_reaction_distance:

                    # Estimate time to reach traffic light at current speed
                    time_to_light = distance_to_light / max(0.1, car.vel[-1])
                    
                    # Predict traffic light state when car would arrive
                    predicted_light_state = tl.predict_light_state(traffic_light, time_to_light)
                    
                    # Handle traffic light based on predicted state
                    if predicted_light_state == "red":

                        # Calculate deceleration needed to stop at light
                        stopping_acc = -car.vel[-1]**2 / (2 * max(distance_to_light, 0.1))
                        safe_decel = max(stopping_acc, -car.comf_decel)
                        acc_new[i] = min(acc_new[i], safe_decel)
                        
                    elif predicted_light_state == "orange":

                        # Decide whether to stop or proceed based on remaining orange time
                        remaining_orange_time = traffic_light.orange_duration - traffic_light.time_in_state
                        
                        if traffic_light.current_state == "orange":
                            time_to_red = remaining_orange_time
                        else:

                            # If green now but will be orange when we arrive
                            time_to_red = (traffic_light.green_duration - traffic_light.time_in_state) + traffic_light.orange_duration
                        
                        # Add a safety buffer
                        safety_buffer = 1.0
                        
                        if time_to_light > time_to_red - safety_buffer:

                            # Can't make it through safely, calculate deceleration to stop
                            stopping_acc = -car.vel[-1]**2 / (2 * max(distance_to_light, 0.1))
                            safe_decel = max(stopping_acc, -car.comf_decel)
                            acc_new[i] = min(acc_new[i], safe_decel)
                        else:

                            # Can make it through
                            min_speed_to_cross = distance_to_light / time_to_red
                            
                            if car.vel[-1] < min_speed_to_cross and car.vel[-1] < car.des_speed:

                                # Calculate acceleration needed to pass before red
                                needed_acc = (min_speed_to_cross - car.vel[-1]) / max(time_to_red, 0.1)
                                safe_acc = min(needed_acc, car.acc_max)
                                acc_new[i] = max(acc_new[i], safe_acc)

        # Set new acceleration value
        return acc_new

    def upd_pos_vel(cars, time_step, acc_new):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))

        for i, car in enumerate(cars):

            # Update velocity and position
            velnew[i] = car.vel[-1] + acc_new[i] * time_step
            posnew[i] = car.pos[-1] + car.vel[-1] * time_step + 0.5 * acc_new[i] * time_step**2

            # Ensure velocity does not go negative
            if velnew[i] < 0:
        
                # Calculate time to stop
                if acc_new[i] != 0:
                    t_stop = -car.vel[-1] / acc_new[i]
                else:
                    t_stop = 0

                velnew[i] = car.vel[-1] + acc_new[i] * t_stop
                posnew[i] = car.pos[-1] + car.vel[-1] * t_stop + 0.5 * acc_new[i] * t_stop**2
        
        # Set new position and velocity values
        return posnew, velnew

    def update_headway_dv(cars, L, time_step):
        
        acc = [car.acc[-1] for car in cars]
        
        for  i, car in enumerate(cars): 
            next_car = cars[(i + 1) % len(cars)]
            
            # Compute headway
            headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L
            
            if headway < car.min_gap:

                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap

                # Calculate acceleration using IDM
                acc_new = car.acc_max * (1 - (car.vel[-2] * car.des_speed_inv)**car.acc_exp - (s_star / (headway))**2)
                
                # Update velocity and position
                velnew = car.vel[-2] + acc_new * time_step

                # Ensure velocity does not go negative
                if velnew < 0:

                    # Calculate time to stop
                    if acc_new != 0:
                        t_stop = -car.vel[-2] / acc_new
                    else:
                        t_stop = 0

                    #posnew = car.pos[-1] + car.vel[-1] * t_stop + 0.5 * acc_new * t_stop**2
                    velnew = car.vel[-2] + acc_new * t_stop
                    #car.pos[-1] = posnew % L
                
                acc[i] = acc_new
                car.vel[-1] = velnew

            car.acc.append(acc[i])
            
        
        for i, car in enumerate(cars):
            next_car = cars[(i + 1) % len(cars)]
            headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L

            car.headway.append(headway)
            car.dv.append(car.vel[-1] - next_car.vel[-1])

        return cars