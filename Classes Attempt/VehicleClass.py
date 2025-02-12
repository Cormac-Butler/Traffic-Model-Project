class VehicleClass:

    def __init__(self, car_id, lane, pos, vel, acc, headway, dv):

        self.car_id = car_id
        self.lane = lane
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.headway = headway
        self.dv = dv

    def upd_pos_vel(self, params):
        
        # Extract parameters
        des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t = params[:7]

        # Calculate desired bumper-to-bumper distance (s*)
        s_star = min_gap + max(0, self.vel * time_gap + (self.vel * self.dv) / (2 * (acc_max * comf_decel)**0.5))

        # Calculate acceleration using IDM
        self.acc = acc_max * (1 - (self.vel * des_speed_inv)**acc_exp - (s_star / self.headway)**2)

        # Update velocity and position
        velnew = self.vel + self.acc * del_t
        posnew = self.pos + self.vel * del_t + 0.5 * self.acc * del_t**2

        # Ensure velocity does not go negative
        if velnew < 0:

            # Calculate time to stop
            t_stop = -self.vel / self.acc

            # Update position and velocity to stop at t_stop
            posnew = self.pos + self.vel * t_stop + 0.5 * self.acc * t_stop**2
            velnew = 0

        return posnew, velnew

    def update_cars(cars, N, posnew, velnew, params, L):

        # Extract parameters
        length, min_gap  = params[7], params[4]

        # Update position and velocity
        for i, car in enumerate(cars):

            car.pos = posnew[i] % L
            car.vel = velnew[i]
        
        # Update headway and velocity difference
        for i, car in enumerate(cars):

            # Get next car
            next_car = cars[(i + 1) % N]

            # Calculate headway (rear bumper to rear bumper)
            car.headway = (next_car.pos - car.pos - length) % L

            # Ensure the minimum gap is maintained
            if car.headway < min_gap:
                car.headway = min_gap

                # Adjust the position of the current car to maintain the minimum gap
                car.pos = (next_car.pos - min_gap - length) % L
            
            # Update velocity difference
            car.dv = next_car.vel - car.vel
        
        return cars