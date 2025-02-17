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
    
    def calc_acc(cars, time_step, L):
        acc_new = np.zeros(len(cars))

        for i, car in enumerate(cars):

            next_car = cars[(i + 1) % len(cars)]

            # Calculate desired bumper-to-bumper distance (s*)
            s_star = car.min_gap + max(0, car.vel[-1] * car.time_gap + (car.vel[-1] * car.dv[-1]) / (2 * (car.acc_max * car.comf_decel)**0.5))

            # Calculate acceleration using IDM
            acc_new[i] = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (s_star / (car.headway[-1]))**2)
        
        return acc_new

    def upd_pos_vel(cars, time_step, L, acc_new):

        posnew = np.zeros(len(cars))
        velnew = np.zeros(len(cars))

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
        for i, car in enumerate(cars):
            next_car = cars[(i + 1) % N]

            # Only consider cars in the same lane
            if next_car.lane[-1] == car.lane[-1]:
                
                # Compute headway
                if next_car.pos[-1] > next_car.length:
                    if next_car.pos[-1] > car.pos[-1]:
                        car.headway.append(next_car.pos[-1] - next_car.length - car.pos[-1])
                    else:
                        car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])
                else:
                    car.headway.append(next_car.pos[-1] - next_car.length + L - car.pos[-1])

                # Compute dv (velocity difference)
                car.dv.append(car.vel[-1] - next_car.vel[-1])
            else:

                # If the next car is not in the same lane, set headway and dv to default values
                car.headway.append(float('inf'))  
                car.dv.append(0)  

        return cars

    def mobil_lane_change(cars, car, L, p=0.1, a_thr=0.1, time_step=0.5):
    
        current_lane = car.lane[-1]
        target_lane = 1 - current_lane 

        # Only consider moving to the right lane if in the left lane
        if current_lane == 1:  
            target_lane = 0 
            benefit = VehicleClass.check_benefit(cars, car, L, target_lane, a_thr)
            if not benefit: 
                car.lane.append(target_lane)

                # Update velocity and position based on new acceleration
                car.vel[-1] += car.acc[-1] * time_step
                car.pos[-1] += car.vel[-1] * time_step + 0.5 * car.acc[-1] * time_step**2
                return True
            return False 

        # Check if moving to the right lane is beneficial
        benefit = VehicleClass.check_benefit(cars, car, L, target_lane, a_thr)
        if not benefit:
            return False 

        # Check if the gap in the target lane is safe
        target_ahead = None
        target_behind = None

        for c in cars:
            if c.lane[-1] == target_lane:
                if c.pos[-1] > car.pos[-1]:
                    if target_ahead is None or c.pos[-1] < target_ahead.pos[-1]:
                        target_ahead = c
                else:
                    if target_behind is None or c.pos[-1] > target_behind.pos[-1]:
                        target_behind = c

        if target_ahead is not None:
            gap_ahead = (target_ahead.pos[-1] - car.pos[-1]) % L - target_ahead.length
            if gap_ahead < car.min_gap:
                return False 

        if target_behind is not None:
            gap_behind = (car.pos[-1] - target_behind.pos[-1]) % L - car.length
            if gap_behind < target_behind.min_gap:
                return False 

        # Check the impact on the car behind in the target lane
        if target_behind is not None:
            old_behind_acc = target_behind.acc[-1]
            new_behind_acc = target_behind.acc_max * (1 - (target_behind.vel[-1] * target_behind.des_speed_inv)**target_behind.acc_exp - (target_behind.min_gap / gap_behind)**2)
            if new_behind_acc - old_behind_acc < -p * a_thr:
                return False

        car.lane.append(target_lane)
        car.vel[-1] = car.vel[-2] + car.acc[-1] * time_step
        car.pos[-1] = car.pos[-2] + car.vel[-2] * time_step + 0.5 * car.acc[-1] * time_step**2

        # Ensure velocity does not go negative
        if car.vel[-1] <= 0:

            # Calculate time to stop
            if abs(car.acc[-1]) > 1e-6:
                t_stop = -car.vel[-1] / car.acc[-1]
            else:
                t_stop = 0

            # Update position and velocity to stop at t_stop
            car.pos[-1] = car.pos[-2] + car.vel[-2] * t_stop + 0.5 * car.acc[-1] * t_stop**2
            car.vel[-1] = 0
        return True


    def check_benefit(cars, car, L, target_lane, a_thr):
        
        current_acc = car.acc[-1]

        # Find the car ahead in the current lane
        current_ahead = None
        for c in cars:
            if c.lane[-1] == car.lane[-1] and c.pos[-1] > car.pos[-1]:
                if current_ahead is None or c.pos[-1] < current_ahead.pos[-1]:
                    current_ahead = c

        # Find the car ahead in the target lane
        target_ahead = None
        for c in cars:
            if c.lane[-1] == target_lane and c.pos[-1] > car.pos[-1]:
                if target_ahead is None or c.pos[-1] < target_ahead.pos[-1]:
                    target_ahead = c

        # Calculate new acceleration in the target lane
        if target_ahead is not None:
            gap_ahead = (target_ahead.pos[-1] - car.pos[-1]) % L - target_ahead.length
            new_acc = car.acc_max * (1 - (car.vel[-1] * car.des_speed_inv)**car.acc_exp - (car.min_gap / gap_ahead)**2)
        else:
            new_acc = car.acc_max 

        # Check if the benefit exceeds the threshold
        if new_acc - current_acc >= a_thr:
            car.acc[-1] = new_acc
            return True  
        return False 