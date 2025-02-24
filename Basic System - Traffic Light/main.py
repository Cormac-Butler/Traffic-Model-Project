import simulationFunctions as sf
import multiprocessing as mp
import pickle

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length):

    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)

    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity

if __name__ == "__main__":

    # Model parameters
    max_cars = 60
    road_length = 500
    steps = 10000
    steps_before_measure = 100
    detection_point = road_length / 2
    time_step = 0.5

    # Simulation steps
    start_cars = 2
    end_cars = max_cars
    step_cars = 2

    # List of car counts to simulate
    car_counts = range(start_cars, end_cars + step_cars, step_cars)

    # Run simulations
    with mp.Pool(processes=mp.cpu_count()) as pool:
        results = pool.starmap(run_simulation, [(N, time_step, steps, steps_before_measure, detection_point, road_length) for N in car_counts])

    # Write data to file
    print("Writing file...")

    global_flow, local_flow = [], []
    global_density, local_density = [], []
    global_average_velocity, local_average_velocity = [], []
    cars_positions = []

    for _, car_list, g_flow, g_density, l_flow, l_density, g_avg_vel, l_avg_vel in results:
        global_flow.append(g_flow)
        global_density.append(g_density)
        local_flow.append(l_flow)
        local_density.append(l_density)
        global_average_velocity.append(g_avg_vel)
        local_average_velocity.append(l_avg_vel)
    cars_positions.append([car.pos for car in car_list])

    # Organise data into a dictionary
    simulation_data = {
        "cars_positions": cars_positions,
        "road_length": road_length,
        "global_density": global_density,
        "global_flow": global_flow,
        "local_density": local_density,
        "local_flow": local_flow,
        "global_average_velocity": global_average_velocity,
        "local_average_velocity": local_average_velocity
    }

    # Save using pickle with highest protocol for speed optimisation
    with open('simulation_results_basic_system.pkl', 'wb') as file:
        pickle.dump(simulation_data, file, protocol=pickle.HIGHEST_PROTOCOL)

    print("File writing complete.")