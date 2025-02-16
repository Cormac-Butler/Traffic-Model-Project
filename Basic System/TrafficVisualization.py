import tkinter as tk
import random
import math

class TrafficVisualization:
    def __init__(self, car_objects, road_length, car_size, update_interval, scale_factor=5):
        self.root = tk.Tk()
        self.root.title("Traffic Simulation")

        self.scale_factor = scale_factor
        self.road_radius = (road_length / (2 * math.pi)) * scale_factor
        self.car_size = car_size
        self.update_interval = update_interval
        self.interp_steps = 5

        self.car_objects = car_objects
        self.num_cars = len(car_objects)
        self.current_step = 0
        self.num_steps = len(car_objects[0]) if self.num_cars > 0 else 0
        self.road_length = road_length
        self.center = (self.road_radius + 50, self.road_radius + 50)

        self.canvas = tk.Canvas(self.root, width=2 * self.road_radius + 100, height=2 * self.road_radius + 100)
        self.canvas.pack()

        self.draw_road()
        self.cars = self.create_cars()

        self.smooth_update()
        self.root.mainloop()

    def draw_road(self):
        x0, y0 = 50, 50
        x1 = x0 + 2 * self.road_radius
        y1 = y0 + 2 * self.road_radius
        self.canvas.create_oval(x0, y0, x1, y1, outline="gray", width=2)

    def create_cars(self):
        cars = []
        for i, car_obj in enumerate(self.car_objects):
            pos = car_obj[0] / self.road_length
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
            pos_current = self.car_objects[i][self.current_step] / self.road_length
            pos_next = self.car_objects[i][next_step] / self.road_length

            for j in range(1, self.interp_steps + 1):
                interp_pos = pos_current + (pos_next - pos_current) * (j / self.interp_steps)
                angle = 2 * math.pi * interp_pos
                x = self.center[0] + self.road_radius * math.cos(angle)
                y = self.center[1] + self.road_radius * math.sin(angle)

                self.canvas.coords(car, x - self.car_size, y - self.car_size, x + self.car_size, y + self.car_size)
                self.root.update_idletasks()

        self.current_step += 1
        self.root.after(self.update_interval, self.smooth_update)