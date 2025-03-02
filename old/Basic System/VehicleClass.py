import numpy as np

class VehicleClass:

    def __init__(self, car_id, pos, vel, acc, headway, dv, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

        self.car_id = car_id

        self.pos = [pos]
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
            s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5)) 

            # Calculate acceleration using IDM
            acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)

        # Set new acceleration value
        return acc_new

    def upd_pos_vel(cars, time_step, acc_new, L):

        for i, car in enumerate(cars):

            # Update velocity and position
            vel_new = car.vel[-1] + acc_new[i] * time_step

            t = time_step

            # Ensure velocity does not go negative
            if vel_new < 0:
        
                # Calculate time to stop
                t = -car.vel[-1] / acc_new[i]

                vel_new = car.vel[-1] + acc_new[i] * t

            pos_new = (car.pos[-1] + car.vel[-1] * t + 0.5 * acc_new[i] * t**2) % L

            #Update values
            car.acc.append(acc_new[i])
            car.vel.append(vel_new)
            car.pos.append(pos_new)

        return cars

    def update_headway_dv(cars, L, time_step):
        
        acc = [car.acc[-1] for car in cars]
        
        for  i, car in enumerate(cars): 
            
            index = (i + 1) % len(cars) 
            next_car = cars[index]
            
            # Compute headway
            headway = ((next_car.pos[-1] - next_car.length) % L - car.pos[-1]) % L
            
            if headway < car.min_gap:

                if len(cars) == 60:
                    ...
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