import matplotlib.pyplot as plt
import TrafficVisualization as tv
import pickle

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
    ax.grid()

results = load_results('simulation_results_basic_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity']}

# Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_graph(axes1[0, 0], data['global_density'], data['global_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plot_graph(axes1[0, 1], data['global_density'], data['global_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Global Avg Velocity vs. Density', 'blue')
plot_graph(axes1[0, 2], data['global_flow'], data['global_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Global Flow vs. Velocity', 'blue')
plot_graph(axes1[1, 0], data['local_density'], data['local_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Local Flow vs. Density', 'red')
plot_graph(axes1[1, 1], data['local_density'], data['local_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Local Avg Velocity vs. Density', 'red')
plot_graph(axes1[1, 2], data['local_flow'], data['local_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Local Flow vs. Velocity', 'red')
plt.tight_layout()
plt.savefig('basic_plots.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 250, 5)