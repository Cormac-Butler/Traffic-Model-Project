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

        self.des_speed = speedlim 
        self.des_speed_inv = 1.0 / speedlim

        self.acc_exp = acc_exp
        self.time_gap = time_gap
        self.min_gap = min_gap

        self.comf_decel = comf_decel
        self.acc_max = acc_max
        self.length = length

    def upd_pos_vel(self, L, time_step):

        # Calculate desired bumper-to-bumper distance (s*)
        s_star = self.min_gap + max(0, self.vel[-1] * self.time_gap + (self.vel[-1] * self.dv[-1]) / (2 * (self.acc_max * self.comf_decel)**0.5))

        # Calculate acceleration using IDM 
        self.acc.append(self.acc_max * (1 - (self.vel[-1] * self.des_speed_inv)**self.acc_exp - (s_star / self.headway[-1])**2))

        # Update velocity and position
        velnew = self.vel[-1] + self.acc[-1] * time_step
        posnew = self.pos[-1] + self.vel[-1] * time_step + 0.5 * self.acc[-1] * time_step**2

        # Ensure velocity does not go negative
        if velnew < 0:

            # Calculate time to stop
            t_stop = -self.vel[-1] / self.acc[-1]

            # Update position and velocity to stop at t_stop
            posnew = self.pos[-1] + self.vel[-1] * t_stop + 0.5 * self.acc[-1] * t_stop**2
            velnew = 0
        
        self.pos.append(posnew % L)
        self.vel.append(velnew)

        return self, velnew

    def update_cars(cars, N, L):

        # Update headway and velocity difference
        for i, car in enumerate(cars):

            # Get next car
            next_car = cars[(i + 1) % N]

            # Calculate headway (rear bumper to rear bumper)
            car.headway.append((next_car.pos[-1] - next_car.length - car.pos[-1]) % L)

            # Ensure the minimum gap is maintained
            if car.headway[-1] < car.min_gap:
                car.headway[-1] = car.min_gap

                # Adjust the position of the current car to maintain the minimum gap
                car.pos[-1] = (next_car.pos[-1] - car.min_gap - next_car.length) % L
            
            # Update velocity difference
            car.dv.append(next_car.vel[-1] - car.vel[-1])
        
        return cars