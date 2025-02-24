import matplotlib.pyplot as plt
import TrafficVisualization as tv
import pickle

def extractFile():
    results = []
    with open('simulation_results_basic_system.pkl', 'rb') as file:
        results = pickle.load(file)
    return results

# Extract results from the file
results = extractFile()

cars = results.get('cars_positions', [])
road_length = results.get('road_length', 0)
global_density = results.get('global_density', [])
global_flow = results.get('global_flow', [])
local_density = results.get('local_density', [])
local_flow = results.get('local_flow', [])
global_average_velocity = results.get('global_average_velocity', [])
local_average_velocity = results.get('local_average_velocity', [])

# Create a 2x3 figure layout
fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# Plot 1: Global Flow vs. Global Density
axes[0, 0].plot(global_density, global_flow, 'bo-', label='Global Flow')
axes[0, 0].set_xlabel('Density (cars/km)')
axes[0, 0].set_ylabel('Flow (cars/hour)')
axes[0, 0].legend()
axes[0, 0].grid()

# Plot 2: Local Flow vs. Local Density
axes[1, 0].plot(local_density, local_flow, 'ro-', label='Local Flow')
axes[1, 0].set_xlabel('Density (cars/km)')
axes[1, 0].set_ylabel('Flow (cars/hour)')
axes[1, 0].legend()
axes[1, 0].grid()

# Plot 3: Global Avg Velocity vs. Density
axes[0, 1].plot(global_density, global_average_velocity, 'bo-', label='Global Avg Velocity')
axes[0, 1].set_xlabel('Density (cars/km)')
axes[0, 1].set_ylabel('Average Velocity (m/s)')
axes[0, 1].legend()
axes[0, 1].grid()

# Plot 4: Local Avg Velocity vs. Density
axes[1, 1].plot(local_density, local_average_velocity, 'ro-', label='Local Avg Velocity')
axes[1, 1].set_xlabel('Density (cars/km)')
axes[1, 1].set_ylabel('Average Velocity (m/s)')
axes[1, 1].legend()
axes[1, 1].grid()

# Plot 5: Global Flow vs. Global Velocity
axes[0, 2].plot(global_flow, global_average_velocity, 'bo-', label='Global Flow')
axes[0, 2].set_xlabel('Flow (cars/hour)')
axes[0, 2].set_ylabel('Average Velocity (m/s)')
axes[0, 2].legend()
axes[0, 2].grid()

# Plot 6: Local Flow vs. Local Velocity
axes[1, 2].plot(local_flow, local_average_velocity, 'ro-', label='Local Flow')
axes[1, 2].set_xlabel('Flow (cars/hour)')
axes[1, 2].set_ylabel('Average Velocity (m/s)')
axes[1, 2].legend()
axes[1, 2].grid()

# Adjust layout to prevent overlap
plt.tight_layout()
plt.savefig('graph1.png')

# Create visualisation
tv.TrafficVisualization(cars, road_length, 5, 250, 5)