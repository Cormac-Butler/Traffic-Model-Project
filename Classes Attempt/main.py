import matplotlib.pyplot as plt
import simulationFunctions as sf
import multiprocessing as mp
import statsmodels.api as sm
import matplotlib.animation as animation
import matplotlib.cm as cm
import TrafficVisualization as tv
import numpy as np

def plot_best_fit_lowess(ax, x, y, xlabel, ylabel, title, color):
    ax.scatter(x, y, color='black', marker='x', label='Data Points', alpha=0.25)

    # Apply LOWESS smoothing
    lowess = sm.nonparametric.lowess(y, x)  
    x_smooth, y_smooth = zip(*lowess)

    ax.plot(x_smooth, y_smooth, color=color, linestyle='-', label='LOWESS Fit')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid()

def run_simulation(N, time_step, steps, steps_before_measure, detection_point, road_length):
    
    # Run simulation
    cars, glob_flow, glob_density, loc_flow, loc_dens = sf.Simulate_IDM(N, time_step, steps, steps_before_measure, detection_point, road_length)
    
    # Compute average velocity (glob_avg_velocity, loc_avg_velocity)
    glob_avg_velocity = (glob_flow * 1000) / (glob_density * 3600) if glob_density != 0 else 0
    loc_avg_velocity = (loc_flow * 1000) / (loc_dens * 3600) if loc_dens != 0 else 0

    # Return only the values that are needed
    return N, cars, glob_flow, glob_density, loc_flow, loc_dens, glob_avg_velocity, loc_avg_velocity
'''
def update(cars, frame, scatters, texts):

    # Extract positions and velocities
    x_positions = [car.pos[frame] for car in cars]
    velocities = [car.vel[frame] for car in cars]

    # Create a colormap
    colormap = cm.viridis
    num_cars = len(cars)

    # Update positions and velocities for all cars
    for i in range(num_cars):
        color = colormap(i / num_cars)

        # Update car position (scatter)
        scatters[i].set_offsets([x_positions[i], 0])
        scatters[i].set_facecolor(color)

        # Update the velocity text
        texts[i].set_position([x_positions[i], 1.5])
        texts[i].set_text(f'{velocities[i]:.1f}')

    return scatters + texts

def update_with_cars(frame):
    return update(cars, frame, scatters, texts)
'''

if __name__ == "__main__":
    
    # Model parameters
    max_cars = 50
    road_length = 300
    steps = 10000  
    steps_before_measure = 100  
    speed_limit = 30  
    detection_point = road_length / 2  
    time_step = 0.5

    start_cars = 2
    end_cars = max_cars
    step_cars = 2

    # Define speed limit zones
    speed_limit_zones = [(0, 30), (1000, 20), (2000, 30), (4000, 5), (6000, 20), (8000, 30)]

    # Define traffic light parameters
    #traffic_light = tl(2000, 30, 5, 30)

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

    # Create visualisation
    tv.TrafficVisualization(cars, road_length, road_length / (2 * np.pi), 5, 250, 5)
    '''
    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(10, 2))

    # Initialise scatters and texts
    scatters = [ax.scatter([], [], s=20) for _ in range(len(cars))]
    texts = [ax.text(0, 1.5, '', fontsize=6, ha='center', color='black') for _ in range(len(cars))]

    # Set axis limits
    ax.set_xlim(0, road_length)
    ax.set_ylim(-5, 10)
    ax.set_xlabel("Road Position (m)")
    ax.set_title("Traffic Simulation (IDM Model)")

    # Create the animation
    ani = animation.FuncAnimation(fig, update_with_cars, frames=steps, interval=10, blit=True)

    # Show the animation
    plt.show()
    '''


    #'''
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
    #'''
    '''
    # Create a figure with multiple subplots
    fig, axes = plt.subplots(3, 2, figsize=(12, 12))

    # Plot each graph in a subplot
    plot_best_fit_lowess(axes[0, 0], global_density, global_flow, 'Density (cars/km)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
    plot_best_fit_lowess(axes[0, 1], global_flow, global_average_velocity, 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Flow vs. Velocity (Global)', 'blue')
    plot_best_fit_lowess(axes[1, 0], local_density, local_flow, 'Density (cars/km)', 'Flow (cars/hour)', 'Local Flow vs. Density', 'red')
    plot_best_fit_lowess(axes[1, 1], global_density, global_average_velocity, 'Density (cars/km)', 'Average Velocity (m/s)', 'Global Avg Velocity vs. Density', 'blue')
    plot_best_fit_lowess(axes[2, 0], local_density, local_average_velocity, 'Density (cars/km)', 'Average Velocity (m/s)', 'Local Avg Velocity vs. Density', 'red')

    # Hide empty subplot
    axes[2, 1].axis('off')

    # Adjust layout and show all plots in one window
    plt.tight_layout()
    plt.show()
    '''