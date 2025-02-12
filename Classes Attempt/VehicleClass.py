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

    def get_local_speed_limit(self, speed_limit_zones):

        for zone_start, speed_limit in speed_limit_zones:
            if self.pos[-1] >= zone_start:
                self.speedlim = speed_limit
                self.des_speed = speed_limit
                self.des_speed_inv = 1.0 / speed_limit

        return self.des_speed
    
    def get_traffic_light_speed_limit(self, traffic_light, L):

        distance_to_light = (traffic_light.position - self.pos[-1]) % L

        if traffic_light.state == "red":
            self.des_speed = 0
            self.des_speed_inv = float('inf')
        elif traffic_light.state == "orange":
            if self.vel[-1] > 0:
                time_to_light = distance_to_light / self.vel[-1]
            else:
                time_to_light = float('inf')

            if time_to_light > traffic_light.orange_duration:
                self.des_speed = 0
                self.des_speed_inv = float('inf')
            else:
                self.des_speed = self.speedlim
                self.des_speed_inv = 1.0 / self.speedlim if self.speedlim > 0 else float('inf')
        else:
            self.des_speed = self.speedlim
            self.des_speed_inv = 1.0 / self.speedlim if self.speedlim > 0 else float('inf')

    def upd_pos_vel(self, L, time_step, speed_limit_zones, traffic_light):

        # Get the local speed limit based on the car's current position
        #self.get_local_speed_limit(speed_limit_zones)

        # Get the local speed limit based on the car's current position
        #self.get_traffic_light_speed_limit(traffic_light, L)

        # Calculate desired bumper-to-bumper distance (s*)
        s_star = self.min_gap + max(0, self.vel[-1] * self.time_gap + (self.vel[-1] * self.dv[-1]) / (2 * (self.acc_max * self.comf_decel)**0.5))

        # Calculate acceleration using IDM
        acc_new = self.acc_max * (1 - (self.vel[-1] * self.des_speed_inv)**self.acc_exp - (s_star / self.headway[-1])**2)
        self.acc.append(acc_new)

        # Update velocity and position
        velnew = self.vel[-1] + acc_new * time_step
        posnew = self.pos[-1] + self.vel[-1] * time_step + 0.5 * acc_new * time_step**2

        # Ensure velocity does not go negative
        if velnew < 0:
            # Calculate time to stop
            if abs(acc_new) > 1e-6:  # Avoid division by zero
                t_stop = -self.vel[-1] / acc_new
            else:
                t_stop = 0

            # Ensure t_stop is valid
            if not np.isfinite(t_stop):
                t_stop = 0

            # Update position and velocity to stop at t_stop
            posnew = self.pos[-1] + self.vel[-1] * t_stop + 0.5 * acc_new * t_stop**2
            velnew = 0

        # Ensure posnew is valid
        if not np.isfinite(posnew):
            posnew = self.pos[-1]

        self.pos.append(posnew % L)
        self.vel.append(velnew)

        return self, velnew

    def update_cars(cars, N, L):

        # Update headway and velocity difference
        for i, car in enumerate(cars):

            # Get next car
            next_car = cars[(i + 1) % N]

            # Calculate headway (front bumper to front bumper)
            car.headway.append((next_car.pos[-1] - car.pos[-1]) % L)

            # Ensure the minimum gap is maintained
            if car.headway[-1] < (car.min_gap + next_car.length):
                car.headway[-1] = car.min_gap + next_car.length

                # Adjust the position of the current car to maintain the minimum gap
                car.pos[-1] = (next_car.pos[-1] - car.min_gap - next_car.length) % L
            
            # Update velocity difference
            car.dv.append(next_car.vel[-1] - car.vel[-1])
        
        return cars