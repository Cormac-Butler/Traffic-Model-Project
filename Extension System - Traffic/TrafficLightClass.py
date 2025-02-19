from VehicleClass import VehicleClass as vc

class TrafficLightClass:
    def __init__(self, position, green_duration, orange_duration, red_duration, time_in_state):
        self.position = position
        self.green_duration = green_duration
        self.orange_duration = orange_duration
        self.red_duration = red_duration
        self.time_in_state = time_in_state

    def status(self, time):
        cycle_time = self.green_duration + self.orange_duration + self.red_duration
        phase = time % cycle_time

        if phase < self.green_duration:
            return "green"
        
        elif phase < self.green_duration + self.orange_duration:
            return "orange"
        else:
            return "red"