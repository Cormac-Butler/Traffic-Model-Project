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
    
    def predict_light_state(traffic_light, time_ahead):

        # Calculate total cycle time
        cycle_time = traffic_light.green_duration + traffic_light.orange_duration + traffic_light.red_duration
        
        # Calculate time in cycle when car will arrive
        current_time_in_cycle = traffic_light.time_in_state

        if traffic_light.current_state == "green":
            current_cycle_time = current_time_in_cycle
        elif traffic_light.current_state == "orange":
            current_cycle_time = traffic_light.green_duration + current_time_in_cycle
        else: 
            current_cycle_time = traffic_light.green_duration + traffic_light.orange_duration + current_time_in_cycle
        
        # Calculate future time in cycle
        future_cycle_time = (current_cycle_time + time_ahead) % cycle_time
        
        # Determine future state
        if future_cycle_time < traffic_light.green_duration:
            return "green"
        elif future_cycle_time < traffic_light.green_duration + traffic_light.orange_duration:
            return "orange"
        else:
            return "red"