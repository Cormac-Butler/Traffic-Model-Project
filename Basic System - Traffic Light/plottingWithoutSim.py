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
    ax.grid(True)

results = load_results('simulation_results_basic_system.pkl')
data, data2 = results

# Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_graph(axes1[0, 0], data['green_durations'], data['global_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

# Basic plots
plot_graph(axes1[0, 1], data2['red_durations2'], data2['global_flow2'], 'Red Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

plot_graph(axes1[1, 0], data['green_durations'], data['local_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

# Basic plots
plot_graph(axes1[1, 1], data2['red_durations2'], data2['local_flow2'], 'Red Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

flow = [data2['local_flow2'][i] + data['local_flow'][i] for i in range(len(data['green_durations']))]

# Basic plots
plot_graph(axes1[0, 2], data['green_durations'], flow, 'Green Duration Road 1 (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

plot_graph(axes1[1, 2], data['green_durations'], data['local_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plot_graph(axes1[1, 2], data2['red_durations2'], data2['local_flow2'], 'Red Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')

plt.savefig('durations_plot2.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 10, 5)