import matplotlib.pyplot as plt
import TrafficVisualization as tv
import pickle

def load_results(filename):

    with open(filename, 'rb') as file:
        return pickle.load(file)

def plot_graph(ax, x, y, xlabel, ylabel, color, marker='o-'):

    #Plot graph
    ax.plot(x, y, marker, color=color, markersize=15, lw=3)
    ax.set_xlabel(xlabel, fontsize=50)
    ax.set_ylabel(ylabel, fontsize=50)
    ax.set_xticklabels(ax.get_xticks(), fontsize=50)
    ax.set_yticklabels(ax.get_yticks(), fontsize=50)
    
    ax.grid()

results = load_results('simulation_results_basic_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity']}

# Basic plots
fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['global_density'], data['global_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'blue')
plt.savefig('gfd.png')

fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['global_density'], data['global_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'blue')
plt.savefig('gvd.png')

fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['global_flow'], data['global_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'blue')
plt.savefig('gvf.png')

fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['local_density'], data['local_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'red')
plt.savefig('lfd.png')

fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['local_density'], data['local_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'red')
plt.savefig('lvd.png')

fig1, axes1 = plt.subplots(1, 1, figsize=(27, 20))
plot_graph(axes1, data['local_flow'], data['local_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'red')
plt.savefig('lvf.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 250, 5)