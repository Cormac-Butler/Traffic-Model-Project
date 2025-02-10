import matplotlib.pyplot as plt
import simulationFunctions as sf

if __name__ == "__main__":

    # Set up model parameters and road parameters
    Nmax, L, steps, steps_measure, splim, params, det_point = sf.init_params()
    
    # Number of different simulations we will run
    start_cars = 2
    end_cars   = 50
    step_cars  = 2 

    # Initialise graph variables
    global_flow   = []
    local_flow    = []

    global_density   = []
    local_density    = []

    global_average_velocity = []
    local_average_velocity  = []

    # Run simulation
    for N in range(start_cars, end_cars + step_cars, step_cars):
        glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, params, steps, steps_measure, det_point)
        global_flow.append(glob_flow)
        global_density.append(glob_density)
        local_flow.append(loc_flow)
        local_density.append(loc_dens)
        global_average_velocity.append((glob_flow*1000)/(glob_density * 3600))

    # Plot global flow vs. global density
    plt.figure()
    plt.plot(global_density, global_flow, 'bo-', label='Global Flow')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Flow (cars/hour)')
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

    # Plot average velocity vs. density (global and local)
    plt.figure()
    plt.plot(global_density, global_average_velocity, 'bo-', label='Global Average Velocity')
    plt.xlabel('Density (cars/km)')
    plt.ylabel('Average Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.show()