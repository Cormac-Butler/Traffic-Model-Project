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

    def check_car_ahead(car, i, cars, L, acc_new) -> None:
        index = (i + 1) % len(cars)
        next_car = cars[index]
        
        # Skip placeholder cars
        if next_car.car_id == -1:
            index = (i + 2) % len(cars)
            next_car = cars[index]
        
        # If next car is stopped or too close
        if next_car.vel[-1] < 0.1:
            safe_gap = car.min_gap + next_car.length
            distance_to_next = (next_car.pos[-1] - car.pos[-1]) % L
            
            if distance_to_next < car.comf_stopping_distance + safe_gap:
                stop_distance = distance_to_next - safe_gap
                if stop_distance > 0:
                    decel = car.vel[-1]**2 / (2 * stop_distance)
                    acc_new[i] = -min(decel, car.comf_decel)
                else:
                    acc_new[i] = -car.comf_decel
                    
    def handle_red_light(car, i, cars, dist_to_light, L, acc_new, traffic_light) -> None:

        # If approaching the light and within stopping distance
        if 0 < dist_to_light < car.comf_stopping_distance + car.length:
            # Calculate deceleration needed to stop at light
            if dist_to_light > car.min_gap:
                stop_distance = dist_to_light - car.min_gap
                if stop_distance > 0:
                    decel = car.vel[-1]**2 / (2 * stop_distance)
                    acc_new[i] = -min(decel, car.comf_decel)
                else:
                    acc_new[i] = -car.comf_decel
            else:
                # Already too close, emergency stop
                acc_new[i] = -car.comf_decel
        
        # Check for stopped car ahead
        VehicleClass.check_car_ahead(car, i, cars, L, acc_new)

    def handle_orange_light(car, i, cars, dist_to_light, L, acc_new, traffic_light) -> None:

        # Calculate time until red
        time_till_red = traffic_light.orange_duration - traffic_light.time_in_state
        
        # If car can make it through before red
        if dist_to_light < car.vel[-1] * time_till_red:

            # Car can pass through safely, maintain speed
            return
        
        # Otherwise, treat like red light - start slowing
        if 0 < dist_to_light < car.comf_stopping_distance + car.length:
            stop_distance = dist_to_light - car.min_gap
            if stop_distance > 0:
                decel = car.vel[-1]**2 / (2 * stop_distance)
                acc_new[i] = -min(decel, car.comf_decel)
            else:
                acc_new[i] = -car.comf_decel
        
        # Check for stopped car ahead
        VehicleClass.check_car_ahead(car, i, cars, L, acc_new)

    
    def upd_pos_vel(cars, time_step, L, traffic_light):
        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))
        
        light_status = traffic_light.status()
        
        # First pass: calculate base accelerations using IDM
        for i, car in enumerate(cars):
            if car.car_id != -1:
                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + 
                                        (car.vel[-1] * car.dv[-1]) / 
                                        (2 * np.sqrt(car.acc_max * car.comf_decel)))

                # Calculate acceleration using IDM
                acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - 
                                        (s_star / car.headway[-1])**2)
        
        # Second pass: handle traffic light logic
        for i, car in enumerate(cars):
            if car.car_id != -1:
                # Distance to traffic light (circular road)
                dist_to_light = (traffic_light.position - car.pos[-1]) % L
                
                # Handle traffic light based on its state
                if light_status == "red":
                    VehicleClass.handle_red_light(car, i, cars, dist_to_light, L, acc_new, traffic_light)
                elif light_status == "orange":
                    VehicleClass.handle_orange_light(car, i, cars, dist_to_light, L, acc_new, traffic_light)
                
                # Calculate new velocity and position based on acceleration
                velnew[i] = max(0, car.vel[-1] + acc_new[i] * time_step)
                posnew[i] = car.pos[-1] + car.vel[-1] * time_step + 0.5 * acc_new[i] * time_step**2

                # Ensure velocity does not go negative
                if velnew[i] <= 0:

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
        
        for  i, car in enumerate(cars):
            if car.car_id != -1: 
                next_car = cars[(i + 1) % N]
                
                # Compute headway
                if next_car.pos[-1] > next_car.length:
                    if next_car.pos[-1] > car.pos[-1]:
                        car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                    else:
                        car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])

                if car.headway[-1] < car.min_gap:

                    # Calculate desired bumper-to-bumper distance (s*)
                    s_star = 2

                    # Calculate acceleration using IDM
                    acc_new = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)
                    
                    # Update velocity and position
                    velnew = car.vel[-1] + acc_new * time_step

                    # Ensure velocity does not go negative
                    if velnew <= 0:
                        velnew = 0
                    
                    car.acc[-1] = acc_new
                    car.vel[-1] = velnew

                car.dv.append(car.vel[-1] - next_car.vel[-1])

        return cars