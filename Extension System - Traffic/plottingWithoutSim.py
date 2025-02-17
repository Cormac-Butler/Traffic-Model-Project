import matplotlib.pyplot as plt
import statsmodels.api as sm
import TrafficVisualization as tv
import pickle
from TrafficLightClass import TrafficLightClass as tl

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

def extractFile():
    results = []
    with open('simulation_results_traffic_extension.pkl', 'rb') as file:
        results = pickle.load(file)
    return results

# Extract results from the file
results = extractFile()

cars = results['cars_positions']
traffic_light = results['traffic_light']
road_length = results['road_length']
global_density = results['global_density']
global_flow = results['global_flow']
local_density = results['local_density']
local_flow = results['local_flow']
global_average_velocity = results['global_average_velocity']
local_average_velocity = results['local_average_velocity']

# Create a 2x3 figure layout
fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# Plot 1: Global Flow vs. Global Density
axes[0, 0].plot(global_density, global_flow, 'bo-', label='Global Flow')
axes[0, 0].set_xlabel('Density (cars/km)')
axes[0, 0].set_ylabel('Flow (cars/hour)')
axes[0, 0].legend()
axes[0, 0].grid()

# Plot 2: Local Flow vs. Local Density
axes[0, 1].plot(local_density, local_flow, 'ro-', label='Local Flow')
axes[0, 1].set_xlabel('Density (cars/km)')
axes[0, 1].set_ylabel('Flow (cars/hour)')
axes[0, 1].legend()
axes[0, 1].grid()

# Plot 3: Global Avg Velocity vs. Density
axes[0, 2].plot(global_density, global_average_velocity, 'bo-', label='Global Avg Velocity')
axes[0, 2].set_xlabel('Density (cars/km)')
axes[0, 2].set_ylabel('Average Velocity (m/s)')
axes[0, 2].legend()
axes[0, 2].grid()

# Plot 4: Local Avg Velocity vs. Density
axes[1, 0].plot(local_density, local_average_velocity, 'ro-', label='Local Avg Velocity')
axes[1, 0].set_xlabel('Density (cars/km)')
axes[1, 0].set_ylabel('Average Velocity (m/s)')
axes[1, 0].legend()
axes[1, 0].grid()

# Plot 5: Global Flow vs. Global Velocity
axes[1, 1].plot(global_flow, global_average_velocity, 'bo-', label='Global Flow')
axes[1, 1].set_xlabel('Flow (cars/hour)')
axes[1, 1].set_ylabel('Average Velocity (m/s)')
axes[1, 1].legend()
axes[1, 1].grid()

# Plot 6: Local Flow vs. Local Velocity
axes[1, 2].plot(local_flow, local_average_velocity, 'ro-', label='Local Flow')
axes[1, 2].set_xlabel('Flow (cars/hour)')
axes[1, 2].set_ylabel('Average Velocity (m/s)')
axes[1, 2].legend()
axes[1, 2].grid()

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()

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

# Create visualisation
tv.TrafficVisualization(cars, road_length, 5, 250, traffic_light, 5)