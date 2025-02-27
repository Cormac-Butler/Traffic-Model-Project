class TrafficLightClass:
    def __init__(self, position, green_duration, orange_duration, red_duration, initial_state="green"):
        self.position = position
        self.green_duration = green_duration
        self.orange_duration = orange_duration
        self.red_duration = red_duration
        self.current_state = initial_state
        self.time_in_state = 0
        self.total_time = 0

    def update(self, time_step):
        self.time_in_state += time_step
        self.total_time += time_step
        
        if self.current_state == "green" and self.time_in_state >= self.green_duration:
            self.current_state = "orange"
            self.time_in_state = 0
        elif self.current_state == "orange" and self.time_in_state >= self.orange_duration:
            self.current_state = "red"
            self.time_in_state = 0
        elif self.current_state == "red" and self.time_in_state >= self.red_duration:
            self.current_state = "green"
            self.time_in_state = 0
            
    def status(self):
        return self.current_state

    def time_in_current_state(self):
        return self.time_in_state