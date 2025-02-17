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
    
    def upd_pos_vel(cars, time_step, L):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))
        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):

            next_car = cars[(i + 1) % len(cars)]

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
        #'''
        while True:
            changes = False

            # Loop to adjust positions for safety gaps
            for i in range(N):
                car = cars[i]
                next_car = cars[(i + 1) % N]
                displacement = 0 

                # Get minimum safety gap
                min_safe_gap = car.min_gap - next_car.length

                # Compute headway
                if next_car.pos[-1] >= next_car.length + car.min_gap:
                    if next_car.pos[-1] > car.pos[-1]:
                        car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                    else:
                        car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
                elif next_car.pos[-1] > next_car.length:
                    if next_car.pos[-1] > car.pos[-1]:
                        car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                    else:
                        car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
                elif next_car.pos[-1] - next_car.length < 0:
                    car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])

                if car.headway[-1] < min_safe_gap:
                    
                    changes = True
                    car.headway[-1] = min_safe_gap

                    # Adjust the position of the current car to maintain the minimum gap
                    if car.pos[-1] > next_car.pos[-1] - min_safe_gap:
                        car.pos[-1] = (next_car.pos[-1] + L - min_safe_gap)
                    else:
                        car.pos[-1] = (next_car.pos[-1] - min_safe_gap)

                    if car.pos[-1] >= car.pos[-2]:
                        displacement = car.pos[-1] - car.pos[-2]
                    else:
                        displacement = (car.pos[-1] + L) - car.pos[-2]

                    car.acc[-1] = 2 * (displacement - car.vel[-2] * time_step) / time_step**2
                    car.vel[-1] = 2 * displacement / time_step - car.vel[-2]
            
            if not changes:
                break
        #'''

        for  i, car in enumerate(cars): 
            next_car = cars[(i + 1) % N]
            '''#next_car_back = (next_car.pos[-1] - next_car.length) % L
            #car_front = car.pos[-1]  # Front bumper of current car
            
            # Compute headway correctly
            #headway = (next_car_back - car_front) % L  # Ensures wraparound works

            #car.headway.append(headway)

            #car.headway.append((next_car.pos[-1] - next_car.length - car.pos[-1]))


            #car.dv.append(car.vel[-1] - next_car.vel[-1])

            # Calculate the position of the rear bumper of the next car
            next_car_rear = (next_car.pos[-1] - next_car.length) % L

            # Calculate the headway
            headway = (next_car_rear - car.pos[-1]) % L

            # Compute headway
            if next_car.pos[-1] >= next_car.length + car.min_gap:
                if next_car.pos[-1] > car.pos[-1]:
                    car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
            elif next_car.pos[-1] > next_car.length:
                if next_car.pos[-1] > car.pos[-1]:
                    car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
            elif next_car.pos[-1] - next_car.length < 0:
                car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
        
            if car.headway[-1] < 1.9:
                 print(car.car_id, car.pos[-1], next_car.pos[-1], car.headway[-1])
            #if (next_car.pos[-1] - next_car.length) % L < car.pos[-1]:
            #    print(car.car_id, car.pos[-1], next_car.pos[-1], car.headway[-1])
            # Ensure the headway is not less than the minimum gap
            min_safe_gap = car.min_gap
            if headway < min_safe_gap:
                headway = min_safe_gap

                # Adjust the position of the current car to maintain the minimum gap
                car.pos[-1] = (next_car_rear - min_safe_gap) % L

            # Update headway and velocity difference
            car.headway.append(headway)'''
            car.dv.append(car.vel[-1] - next_car.vel[-1])

        return cars