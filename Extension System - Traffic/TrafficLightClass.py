from VehicleClass import VehicleClass as vc

class TrafficLightClass:
    def __init__(self, position, green_duration, orange_duration, red_duration):
        self.position = position
        self.green_duration = green_duration
        self.orange_duration = orange_duration
        self.red_duration = red_duration

    def status(self, time):
        cycle_time = self.green_duration + self.orange_duration + self.red_duration
        phase = time % cycle_time

        if phase < self.red_duration:
            return "red"
        elif phase < self.red_duration + self.orange_duration:
            return "orange"
        else:
            return "green"