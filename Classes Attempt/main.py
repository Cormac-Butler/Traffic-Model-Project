import matplotlib.pyplot as plt
import simulationFunctions as sf

if __name__ == "__main__":

    # Model parameters
    max_cars = 100  # Maximum number of cars
    road_length = 6000  # Length of ring road (meters)
    steps = 1000  # Total number of steps
    steps_before_measure = 100  # Steps before we start to measure
    speed_limit = 30  # Speed limit in m/s (approx 108 km/h)
    detection_point = road_length / 2  # Detection point in meters
    time_step = 0.5

    # Number of different simulations we will run
    start_cars = 100
    end_cars = 1000
    step_cars = 100

    # Initialize graph variables
    global_flow = []
    local_flow = []
    global_density = []
    local_density = []
    global_average_velocity = []
    local_average_velocity  = []

    # Run simulation
    for N in range(start_cars, end_cars + step_cars, step_cars):
        glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)
        
        global_flow.append(glob_flow)
        global_density.append(glob_density)
        
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        
        global_average_velocity.append((glob_flow * 1000) / (glob_density * 3600))
        #local_average_velocity.append((loc_flow * 1000) / (loc_dens * 3600))

    # Plot global flow vs. global density
    plt.figure()
    plt.plot(global_density, global_flow, 'bo-', label='Global Flow')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Flow (cars/hour)')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot global flow vs. global density
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