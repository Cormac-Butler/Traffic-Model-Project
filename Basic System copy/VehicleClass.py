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
    
    def upd_pos_vel(cars, time_step, L):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):

            # Calculate desired bumper-to-bumper distance (s*)
            s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

            # Calculate acceleration using IDM
            acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)

        for i, car in enumerate(cars):

            # Update velocity and position
            velnew[i] = car.vel[-1] + acc_new[i] * time_step
            posnew[i] = car.pos[-1] + car.vel[-1] * time_step + 0.5 * acc_new[i] * time_step**2

            # Ensure velocity does not go negative
            if velnew[i] <= 0:

                # Calculate time to stop
                if acc_new[i] != 0:
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
        acc = [car.acc[-1] for car in cars]
        
        for  i, car in enumerate(cars): 
            next_car = cars[(i + 1) % N]
            
            # Compute headway
            car.headway.append(((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L)
            
            if car.headway[-1] < car.min_gap:

                # Calculate desired bumper-to-bumper distance (s*)
                s_star = car.min_gap

                # Calculate acceleration using IDM
                acc_new = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)
                
                # Update velocity and position
                velnew = car.vel[-1] + acc_new * time_step

                # Ensure velocity does not go negative
                if velnew < 0:
                    velnew = 0
                
                acc[i] = acc_new
                car.vel[-1] = velnew

            car.acc.append(acc[i])
        
        for i, car in enumerate(cars):
            next_car = cars[(i + 1) % N]
            car.dv.append(car.vel[-1] - next_car.vel[-1])

        return cars