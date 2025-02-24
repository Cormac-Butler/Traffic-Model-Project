import simulationFunctions as sf
import multiprocessing as mp
import pickle
from TrafficLightClass import TrafficLightClass as tl

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length, traffic_light):
    
    # Run simulation
    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length, traffic_light)
    
    # Compute average velocity (glob_avg_velocity, loc_avg_velocity)
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    # Return only the values that are needed
    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity

if __name__ == "__main__":
    
    # Model parameters
    max_cars = 20
    road_length = 500
    steps = 10000
    steps_before_measure = 100 
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 2
    end_cars = max_cars
    step_cars = 2

    # Define traffic light parameters
    traffic_light = tl(100, 10, 5, 10)

    # List of car counts to simulate
    car_counts = range(start_cars, end_cars + step_cars, step_cars)
    cars = []

    # Parallel processing using multiprocessing Pool
    with mp.Pool(processes=mp.cpu_count()) as pool:
        results = pool.starmap(run_simulation, [(N, time_step, steps, steps_before_measure, detection_point, road_length, traffic_light) for N in car_counts])

    # Unpack results
    global_flow, local_flow = [], []
    global_density, local_density = [], []
    global_average_velocity, local_average_velocity = [], []
    cars_positions = [0] * max_cars

    for _, carMax, g_flow, g_density, l_flow, l_density, g_avg_vel, l_avg_vel in results:
        global_flow.append(g_flow)
        global_density.append(g_density)
        local_flow.append(l_flow)
        local_density.append(l_density)
        global_average_velocity.append(g_avg_vel)
        local_average_velocity.append(l_avg_vel)
        cars = carMax

    cars_positions = [car.pos for car in cars]
    
    # Organise data into a dictionary
    simulation_data = {
        "traffic_light": traffic_light, 
        "cars_positions": cars_positions,
        "road_length": road_length,
        "global_density": global_density,
        "global_flow": global_flow,
        "local_density": local_density,
        "local_flow": local_flow,
        "global_average_velocity": global_average_velocity,
        "local_average_velocity": local_average_velocity
    }

    # Save to a pickle file
    with open('simulation_results_traffic_extension.pkl', 'wb') as file:
        pickle.dump(simulation_data, file,  protocol=pickle.HIGHEST_PROTOCOL)