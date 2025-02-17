import simulationFunctions as sf
import multiprocessing as mp
from TrafficLightClass import TrafficLightClass as tl
import pickle

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
    max_cars = 2
    road_length = 300
    steps = 1000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 2
    end_cars = max_cars
    step_cars = 2

    # Define traffic light parameters
    traffic_light = tl(100, 15, 0, 10)

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

    for res in results:
        N, carsMaxN, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity = res

        global_flow.append(glob_flow)
        global_density.append(glob_density)
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        global_average_velocity.append(glob_avg_velocity)
        local_average_velocity.append(loc_avg_velocity)
        cars = carsMaxN

    # OrganiSe results into a dictionary
    simulation_data = {
        "traffic_light": traffic_light, 
        "cars_positions": [[car.pos for car in res[1]] for res in results],
        "road_length": road_length,
        "global_density": [res[3] for res in results],
        "global_flow": [res[2] for res in results],
        "local_density": [res[5] for res in results],
        "local_flow": [res[4] for res in results],
        "global_average_velocity": [res[6] for res in results],
        "local_average_velocity": [res[7] for res in results]
    }

    # Save to a pickle file
    with open('simulation_results_traffic_extension.pkl', 'wb') as file:
        pickle.dump(simulation_data, file)