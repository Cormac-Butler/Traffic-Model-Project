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
    
    def upd_pos_vel(cars, time_step):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):

            # Calculate desired bumper-to-bumper distance (s*)
            s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

            # Calculate acceleration using IDM
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

            car.acc.append(acc_new[i])

        return cars, velnew, posnew

    def update_cars(cars, N, L, posnew, velnew):

        # Update position and velocity
        for i, car in enumerate(cars):
            car.vel.append(velnew[i])
            car.pos.append(posnew[i] % L)

        # Update headway and velocity difference
        for i, car in enumerate(cars):

            # Get next car
            next_car = cars[(i + 1) % N]

            # Calculate headway (front bumper to front bumper)

            if next_car.pos[-1] > car.pos[-1]:
                car.headway.append(next_car.pos[-1] - car.pos[-1])
            else:
                car.headway.append((next_car.pos[-1] + L - car.pos[-1]))

            # Ensure the minimum gap is maintained
            if car.headway[-1] < (car.min_gap + next_car.length):
                car.headway[-1] = car.min_gap + next_car.length

                # Adjust the position of the current car to maintain the minimum gap
                car.pos[-1] = (next_car.pos[-1] - car.min_gap - next_car.length) % L
            
            # Update velocity difference
            car.dv.append(next_car.vel[-1] - car.vel[-1])
        
        return cars