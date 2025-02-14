from VehicleClass import VehicleClass as vc

class TrafficLightClass:
    def __init__(self, position, green_duration, orange_duration, red_duration):
        self.position = position
        self.green_duration = green_duration
        self.orange_duration = orange_duration
        self.red_duration = red_duration
        self.state = "green"
        self.time_in_state = 0

    def update(self, time_step):

        self.time_in_state += time_step

        if self.state == "green" and self.time_in_state >= self.green_duration:
            self.state = "orange"
            self.time_in_state = 0
        elif self.state == "orange" and self.time_in_state >= self.orange_duration:
            self.state = "red"
            self.time_in_state = 0
        elif self.state == "red" and self.time_in_state >= self.red_duration:
            self.state = "green"
            self.time_in_state = 0

    def get_phantom_car(self):

        if self.state == "red":

            # Create a phantom car with zero velocity and acceleration
            phantom_car = vc(-1,0, [self.position], [0], [0], [float('inf')], [0], 0, 4, 1, 2, 1.5, 0, 3)

            return phantom_car

        return None