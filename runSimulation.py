import matplotlib.pyplot as plt
import simulationFunctions as sf

if __name__ == "__main__":

    # Set up model parameters and road parameters
    Nmax, L, steps, steps_measure, splim, params = sf.init_params()
    
    # Number of different simulations we will run
    start_cars = 2
    end_cars   = 100 
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

        glob_flow, glob_density = sf.Simulate_IDM(N,params,steps,steps_measure)

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

    print('hi')