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
data, data2 = results

# Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_graph(axes1[0, 0], data['green_durations'], data['global_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plt.savefig('durations_plot.png')

# Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_graph(axes1[0, 0], data['green_durations'], data2['global_flow2'], 'Red Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plt.savefig('durations2_plot.png')

flow = [data2['global_flow2'][i] + data['global_flow'][i] for i in range(len(data['green_durations']))]
# Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_graph(axes1[0, 0], data['green_durations'], flow, 'Red Duration (s)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plt.savefig('durations3_plot.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 10, 5)