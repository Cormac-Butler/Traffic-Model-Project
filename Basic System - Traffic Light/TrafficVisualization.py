import tkinter as tk
import random
import math

class TrafficVisualization:
    def __init__(self, car_objects, road_length, car_size, update_interval, scale_factor=0.5):
        self.root = tk.Tk()
        self.root.title("Traffic Simulation")

        self.scale_factor = scale_factor
        self.road_radius = (road_length / (2 * math.pi)) * scale_factor
        self.car_size = car_size
        self.update_interval = update_interval
        self.interp_steps = 5

        # Handle car_objects structure correctly
        self.car_objects = car_objects
        self.num_cars = len(car_objects)
        self.current_step = 0
        self.num_steps = len(car_objects[0]) if self.num_cars > 0 else 0
        self.road_length = road_length
        self.center = (self.road_radius + 50, self.road_radius + 50)
        
        # Traffic light parameters
        self.step_size = 0.5
        self.orange_duration = 20
        self.green_duration = 40 
        self.red_duration = 60  
        self.traffic_light_state = "green" 
        self.traffic_light_timer = 0
        self.green_durations = [] 
        self.traffic_light_position = 150
        
        # Canvas setup
        canvas_width = 2 * self.road_radius + 100
        canvas_height = 2 * self.road_radius + 100
        self.canvas = tk.Canvas(self.root, width=canvas_width, height=canvas_height)
        self.canvas.pack()

        self.draw_road()
        self.cars = self.create_cars()
        self.traffic_light = self.create_traffic_light()

        self.smooth_update()
        self.root.mainloop()

    def calculate_light_durations(self):

        # Total cycle is 120 seconds
        total_cycle = 120

        # Orange is fixed at 20 seconds
        self.orange_duration = 20

        # Calculate green duration from provided data
        if self.green_durations and len(self.green_durations) > 0:
            self.green_duration = self.green_durations[0]
        else:

            # Default if green_durations not provided
            self.green_duration = 40

        # Red duration is the remainder
        self.red_duration = total_cycle - self.green_duration - self.orange_duration

    def draw_road(self):
        x0, y0 = 50, 50
        x1 = x0 + 2 * self.road_radius
        y1 = y0 + 2 * self.road_radius
        self.canvas.create_oval(x0, y0, x1, y1, outline="gray", width=2)
        
        # Calculate the angle for the traffic light position
        traffic_light_angle = 2 * math.pi * (self.traffic_light_position / self.road_length)
        x = self.center[0] + self.road_radius * math.cos(traffic_light_angle)
        y = self.center[1] + self.road_radius * math.sin(traffic_light_angle)
        
        # Draw traffic light position marker
        self.canvas.create_rectangle(x-5, y-5, x+5, y+5, fill="black")
        
        # Return traffic light angle for positioning the visible traffic light
        return traffic_light_angle

    def create_traffic_light(self):

        # Get the angle for the traffic light position
        traffic_light_angle = 2 * math.pi * (self.traffic_light_position / self.road_length)
        
        # Calculate position for traffic light
        light_x = self.center[0] + (self.road_radius + 20) * math.cos(traffic_light_angle)
        light_y = self.center[1] + (self.road_radius + 20) * math.sin(traffic_light_angle)
        
        # Calculate rotation for traffic light
        rotation_angle = traffic_light_angle + math.pi/2
        
        # Traffic light housing
        housing = self.canvas.create_rectangle(
            light_x - 15, light_y - 45,
            light_x + 15, light_y + 45,
            fill="black", outline="gray"
        )
        
        # Red light
        red_light = self.canvas.create_oval(
            light_x - 10, light_y - 40,
            light_x + 10, light_y - 20,
            fill="gray", outline="gray"
        )
        
        # Orange light
        orange_light = self.canvas.create_oval(
            light_x - 10, light_y - 10,
            light_x + 10, light_y + 10,
            fill="gray", outline="gray"
        )
        
        # Green light - set to active initially
        green_light = self.canvas.create_oval(
            light_x - 10, light_y + 20,
            light_x + 10, light_y + 40,
            fill="green", outline="gray"
        )
        
        return {
            "housing": housing,
            "red": red_light,
            "orange": orange_light,
            "green": green_light,
            "state": "green",
            "timer": 0
        }

    def update_traffic_light(self):
        # Convert current step to time in seconds
        current_time = self.current_step * self.step_size
        
        # Modify cycle to start with green (reverse the normal order)
        cycle_time = current_time % 120
        
        # Green -> Orange -> Red cycle starting with Green
        if cycle_time < self.green_duration:
            new_state = "green"
        elif cycle_time < self.green_duration + self.orange_duration:
            new_state = "orange"
        else:
            new_state = "red"
        
        # Update the traffic light display only if state has changed
        if new_state != self.traffic_light["state"]:
            self.traffic_light["state"] = new_state
            
            # Reset all lights to gray
            self.canvas.itemconfig(self.traffic_light["red"], fill="gray")
            self.canvas.itemconfig(self.traffic_light["orange"], fill="gray")
            self.canvas.itemconfig(self.traffic_light["green"], fill="gray")
            
            # Set the active light
            if new_state == "red":
                self.canvas.itemconfig(self.traffic_light["red"], fill="red")
            elif new_state == "orange":
                self.canvas.itemconfig(self.traffic_light["orange"], fill="orange")
            elif new_state == "green":
                self.canvas.itemconfig(self.traffic_light["green"], fill="green")

    def create_cars(self):
        cars = []
        for i in range(self.num_cars):
            # Get the initial position for this car
            pos = float(self.car_objects[i][0]) / self.road_length
            angle = 2 * math.pi * pos
            x = self.center[0] + self.road_radius * math.cos(angle)
            y = self.center[1] + self.road_radius * math.sin(angle)

            car = self.canvas.create_oval(
                x - self.car_size, y - self.car_size,
                x + self.car_size, y + self.car_size,
                fill=self.get_car_color(i),
                outline="black"
            )
            cars.append(car)
        return cars

    def get_car_color(self, index):
        random.seed(index)
        return f"#{random.randint(0, 255):02x}{random.randint(0, 255):02x}{random.randint(0, 255):02x}"

    def smooth_update(self):
        if self.current_step >= self.num_steps - 1:
            return
        
        next_step = self.current_step + 1

        for i, car in enumerate(self.cars):
            pos_current = float(self.car_objects[i][self.current_step]) / self.road_length
            pos_next = float(self.car_objects[i][next_step]) / self.road_length

            for j in range(1, self.interp_steps + 1):
                interp_pos = pos_current + (pos_next - pos_current) * (j / self.interp_steps)
                angle = 2 * math.pi * interp_pos
                x = self.center[0] + self.road_radius * math.cos(angle)
                y = self.center[1] + self.road_radius * math.sin(angle)

                self.canvas.coords(car, x - self.car_size, y - self.car_size, x + self.car_size, y + self.car_size)
                self.root.update_idletasks()

        # Update traffic light
        self.update_traffic_light()
        
        self.current_step += 1
        self.root.after(self.update_interval, self.smooth_update)

# Main script
import matplotlib.pyplot as plt
import pickle

def load_results(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)

def plot_graph(ax, x, y, xlabel, ylabel, title, color, marker='o-', label=None):
    # Plot graph
    ax.plot(x, y, marker, color=color, label=label or title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid()

# Load data
results = load_results('simulation_results_basic_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity', 'green_durations']}

# Create visualization instance with traffic light
visualization = TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 250, 5)

# Pass green durations to the visualization
visualization.green_durations = data['green_durations']
visualization.calculate_light_durations()