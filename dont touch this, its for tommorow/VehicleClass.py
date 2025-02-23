class VehicleClass:
    def __init__(self, car_id, lane, initial_pos, initial_vel, speedlim, acc_exp, time_gap, min_gap, comf_decel, acc_max, length):

        self.car_id = car_id
        self.lane = lane
        self.pos = [initial_pos]
        self.vel = initial_vel
        self.acc = 0
        self.headway = 0
        self.dv = 0
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