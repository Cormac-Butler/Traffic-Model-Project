import simulationFunctions as sf
import multiprocessing as mp
import pickle
import matplotlib.pyplot as plt
import numpy as np

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length):
    
    # Run simulation
    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)
    
    # Compute average velocity (glob_avg_velocity, loc_avg_velocity)
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    # Return only the values that are needed
    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity

def plot_lane_position_over_time(cars, road_length, steps, time_step):
    """
    Plot the lane and position of vehicles over time.
    :param cars: List of VehicleClass objects.
    :param road_length: Length of the road.
    :param steps: Total number of simulation steps.
    :param time_step: Time step for the simulation.
    """
    plt.figure(figsize=(12, 6))
    
    # Create a time array with steps + 1 elements (including t = 0)
    time = np.arange(0, (steps + 1) * time_step, time_step)
    
    # Define colors for lanes
    lane_colors = {0: 'blue', 1: 'red'}  # Blue for left lane, red for right lane
    
    # Plot each vehicle's position over time, colored by lane
    for car in cars:
        if car.car_id != -1:  # Skip dummy cars
            # Plot the position over time, changing color based on lane
            for t in range(len(time) - 1):
                if t < len(car.lane):  # Ensure we don't exceed the length of the lane list
                    plt.plot(
                        [time[t], time[t + 1]],  # Time segment
                        [car.pos[t], car.pos[t + 1]],  # Position segment
                        color=lane_colors[car.lane[t]],  # Lane color at time t
                        label=f'Car {car.car_id}' if t == 0 else ""  # Label only once
                    )
    
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Vehicle Lane and Position Over Time')
    plt.ylim(0, road_length)  # Set y-axis limits to road length
    plt.grid(True)
    
    # Create a custom legend for lanes
    handles = [plt.Line2D([0], [0], color=lane_colors[0], label='Left Lane (0)'),
               plt.Line2D([0], [0], color=lane_colors[1], label='Right Lane (1)')]
    plt.legend(handles=handles, bbox_to_anchor=(1.05, 1), loc='upper left')  # Place legend outside the plot
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    
    # Model parameters
    max_cars = 10
    road_length = 300
    steps = 1000 
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 2
    end_cars = max_cars
    step_cars = 2

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
    cars_positions = [0] * max_cars

    '''
    for res in results:
        N, carsMaxN, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity = res

        global_flow.append(glob_flow)
        global_density.append(glob_density)
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        global_average_velocity.append(glob_avg_velocity)
        local_average_velocity.append(loc_avg_velocity)
        cars = carsMaxN
    '''

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
        "cars_positions": cars_positions,
        "road_length": road_length,
        "global_density": global_density,
        "global_flow": global_flow,
        "local_density": local_density,
        "local_flow": local_flow,
        "global_average_velocity": global_average_velocity,
        "local_average_velocity": local_average_velocity
    }

    # Save using pickle with highest protocol for speed optimization
    with open('simulation_results_basic_system.pkl', 'wb') as file:
        pickle.dump(simulation_data, file, protocol=pickle.HIGHEST_PROTOCOL)
    
    # Plot vehicle positions over time
    plot_lane_position_over_time(cars, road_length, steps, time_step)