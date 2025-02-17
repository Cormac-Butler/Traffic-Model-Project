import simulationFunctions as sf
import multiprocessing as mp

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length):
    
    # Run simulation
    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)
    
    # Compute average velocity (glob_avg_velocity, loc_avg_velocity)
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    # Return only the values that are needed
    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity

if __name__ == "__main__":
    
    # Model parameters
    max_cars = 600
    road_length = 3200
    steps = 1000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 100
    end_cars = max_cars
    step_cars = 25

    # List of car counts to simulate
    car_counts = range(start_cars, end_cars + step_cars, step_cars)
    cars = []

    # Parallel processing using multiprocessing Pool
    with mp.Pool(processes=mp.cpu_count()) as pool:
        results = pool.starmap(run_simulation, [(N, time_step, steps, steps_before_measure, detection_point, road_length) for N in car_counts])

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
    
    # Package the required values into a text file so we don't have to run the sim 6042 times 
    with open('simulation_results_basic_system.txt', 'w') as file:
        file.write(f"cars_positions: {[car.pos for car in cars]}\n")
        file.write(f"road_length: {road_length}\n")
        file.write(f"global_density: {global_density}\n")
        file.write(f"global_flow: {global_flow}\n")
        file.write(f"local_density: {local_density}\n")
        file.write(f"local_flow: {local_flow}\n")
        file.write(f"global_average_velocity: {global_average_velocity}\n")
        file.write(f"local_average_velocity: {local_average_velocity}\n")

    
    for car in cars:

        print(max(car.vel))