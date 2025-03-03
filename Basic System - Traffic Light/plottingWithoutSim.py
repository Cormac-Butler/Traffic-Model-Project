import matplotlib.pyplot as plt
import TrafficVisualization as tv
import pickle
import numpy as np

def load_results(filename):

    with open(filename, 'rb') as file:
        return pickle.load(file)

def plot_graph(ax, x, y, xlabel, ylabel, title, color, marker='o-', label=None):

    #Plot graph
    ax.plot(x, y, marker, color=color, label=label or title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

results = load_results('simulation_results_basic_system.pkl')

results = load_results('simulation_results_basic_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity', 'green_durations']}

# Basic plots
fig1, axes1 = plt.subplots(2, 1, figsize=(15, 10))
plot_graph(axes1[0], data['green_durations'], data['global_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

plot_graph(axes1[1], data['green_durations'], data['local_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

#grad = np.gradient(flow, data['green_durations'])
#print(np.mean(grad))

plt.savefig('durations_plot2.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 10, 5)