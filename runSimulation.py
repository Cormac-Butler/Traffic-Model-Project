import matplotlib.pyplot as plt
import simulationFunctions

if __name__ == "__main__":

    # Set up model parameters and road parameters
    _, _, steps, steps_measure, _, params = simulationFunctions.init_params()

    # Number of different simulations we will run
    start_cars = 2 # Start with this many
    end_cars   = 100 # End with this many
    step_cars  = 2 # Gap between steps

    # Initialise graph variables
    global_flow   = []
    local_flow    = []

    global_density   = []
    local_density    = []

    global_average_velocity = []
    local_average_velocity  = []

    # Run simulation
    for N in range(start_cars, end_cars + step_cars, step_cars):

        glob_flow, glob_density = simulationFunctions.Simulate_IDM(N,params,steps,steps_measure)

        global_flow.append(glob_flow)
        global_density.append(glob_density)
        global_average_velocity.append((glob_flow*1000)/(glob_density * 3600))

    print(global_average_velocity)

    # Plot graphs
    
    #f1 = plt.figure()
    #plt.plot(global_flow, global_density)
    #f2 = plt.figure()
    #plt.plot(global_flow, global_density)
    f3 = plt.figure()
    plt.plot(global_average_velocity, global_density)
    plt.show()