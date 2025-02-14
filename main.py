import simulationFunctions as sf
import numpy as np
import TrafficVisualization as tv

if __name__ == "__main__":

    np.random.seed(20)
    
    # Model parameters
    max_cars = 500
    road_length = 10000
    steps = 10000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    # Run simulation
    _, _, _, _, _, carsInitial = sf.Simulate_IDM(max_cars, time_step, steps, steps_before_measure, detection_point, road_length)

    # Create visualization
    tv.TrafficVisualization(carsInitial, road_length, 300, 5, 1)