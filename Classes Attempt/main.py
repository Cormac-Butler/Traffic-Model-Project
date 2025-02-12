import matplotlib.pyplot as plt
import simulationFunctions as sf
from TrafficLightClass import TrafficLightClass as tl
import multiprocessing as mp

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length, speed_limit_zones, traffic_light):

    # Run simulation
    glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length, speed_limit_zones, traffic_light)
    
    # Compute average velocity
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600)
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600)

    return N, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity


if __name__ == "__main__":
    
    # Model parameters
    max_cars = 2000  
    road_length = 12000  
    steps = 1000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    # Define speed limit zones
    speed_limit_zones = [(0, 30), (1000, 20), (2000, 30), (4000, 5), (6000, 20), (8000, 30)]

    # Define traffic light parameters
    traffic_light = tl(2000, 30, 5, 30)

    start_cars = 100
    end_cars = max_cars
    step_cars = 100

    # List of car counts to simulate
    car_counts = range(start_cars, end_cars + step_cars, step_cars)

    # Parallel processing using multiprocessing Pool
    with mp.Pool(processes=mp.cpu_count()) as pool:
        results = pool.starmap(run_simulation, [(N, time_step, steps, steps_before_measure, detection_point, road_length, speed_limit_zones, traffic_light) for N in car_counts])

    # Unpack results
    global_flow, local_flow = [], []
    global_density, local_density = [], []
    global_average_velocity, local_average_velocity = [], []

    for res in results:
        N, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity = res

        global_flow.append(glob_flow)
        global_density.append(glob_density)
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        global_average_velocity.append(glob_avg_velocity)
        local_average_velocity.append(loc_avg_velocity)

    # Plot global flow vs. global density
    plt.figure()
    plt.plot(global_density, global_flow, 'bo-', label='Global Flow')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Flow (cars/hour)')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot flow vs. velocity
    plt.figure()
    plt.plot(global_flow, global_average_velocity, 'bo-', label='Global Flow')
    plt.xlabel('Flow (cars/hour)')
    plt.ylabel('Average Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot local flow vs. local density
    plt.figure()
    plt.plot(local_density, local_flow, 'ro-', label='Local Flow')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Flow (cars/hour)')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot average velocity vs. density (global)
    plt.figure()
    plt.plot(global_density, global_average_velocity, 'bo-', label='Global Average Velocity')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Average Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot average velocity vs. density (local)
    plt.figure()
    plt.plot(local_density, local_average_velocity, 'ro-', label='Local Average Velocity')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Average Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.show()
