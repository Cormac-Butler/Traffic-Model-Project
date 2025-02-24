import numpy as np

class VehicleClass:
    def __init__(self, car_id, initial_pos, initial_vel, initial_acc, initial_headway, initial_dv, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

        self.car_id = car_id

        self.pos = [initial_pos]
        self.vel = initial_vel
        self.acc = initial_acc

        self.headway = initial_headway
        self.dv = initial_dv

        self.prev_vel = initial_vel
        self.prev_acc = 0

        self.speedlim = speedlim
        self.des_speed = speedlim
        self.des_speed_inv = 1.0 / speedlim if speedlim > 0 else 0

        self.acc_exp = acc_exp
        self.time_gap = time_gap
        self.min_gap = min_gap
        self.comf_decel = comf_decel
        self.acc_max = acc_max

        self.length = length

    def update_cars(cars, time_step, L):

        # Update acc based on IDM
        acc_new = VehicleClass.calc_acc(cars)

        # Update velocity and position
        cars = VehicleClass.upd_pos_vel(cars, time_step, acc_new, L)
        
        cars = VehicleClass.update_headway_dv(cars, L, time_step)
            
        return cars
    
    def calc_acc(cars):

        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):

            # Calculate desired bumper-to-bumper distance (s*)
            s_star = car.min_gap + max(0, car.vel * car.time_gap + (car.vel * car.dv) / (2 * (car.acc_max * car.comf_decel)**0.5))

            # Calculate acceleration using IDM
            acc_new[i] = car.acc_max * (1 - (car.vel * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway))**2)

        # Set new acceleration value
        return acc_new
    
    def upd_pos_vel(cars, time_step, acc_new, L):

        for i, car in enumerate(cars):

            # Update velocity and position
            vel_new = car.vel + acc_new[i] * time_step

            t = time_step

            # Ensure velocity does not go negative
            if vel_new < 0:
        
                # Calculate time to stop
                t = -car.vel / acc_new[i]

                vel_new = car.vel + acc_new[i] * t
            
            pos_new = (car.pos[-1] + car.vel * t + 0.5 * acc_new[i] * t**2) % L

            # Update values
            car.prev_vel = car.vel
            car.prev_acc = acc_new[i]
            car.acc = acc_new[i]
            car.vel = vel_new
            car.pos.append(pos_new)

        return cars
    
    def update_headway_dv(cars, L, time_step):
    
        for  i, car in enumerate(cars): 
            
            index = (i + 1) % len(cars) 
            next_car = cars[index]
            
            # Compute headway
            headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L
            
            if headway < car.min_gap:

                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap

                # Calculate acceleration using IDM
                acc_new = car.acc_max * (1 - (car.prev_vel * car.des_speed_inv)**car.acc_exp - (s_star / (headway))**2)
                
                # Update velocity and position
                vel_new = car.prev_vel + acc_new * time_step

                # Ensure velocity does not go negative
                if vel_new < 0:

                    # Calculate time to stop
                    t_stop = -car.prev_vel / acc_new

                    vel_new = car.prev_vel + acc_new * t_stop
                
                # Update acceleration and velocity
                car.acc = acc_new
                car.vel = vel_new
        
        for i, car in enumerate(cars):
            next_car = cars[(i + 1) % len(cars)]
            headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L

            car.headway = headway
            car.dv = car.vel - next_car.vel

        return cars