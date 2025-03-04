import matplotlib.pyplot as plt
import TrafficVisualization as tv
import pickle
from statsmodels.nonparametric.smoothers_lowess import lowess

def load_results(filename):

    with open(filename, 'rb') as file:
        return pickle.load(file)

def plot_graph(ax, x, y, xlabel, ylabel, color, marker='0'):

    # Scatter plot
    ax.scatter(x, y, color=color, marker='x')

    # LOWESS smoothing for best-fit line
    smoothed = lowess(y, x, frac=0.32) 
    ax.plot(smoothed[:, 0], smoothed[:, 1], 'r-', lw=3)

    # Formatting
    ax.set_xlabel(xlabel, fontsize=20)
    ax.set_ylabel(ylabel, fontsize=20)
    ax.grid(True)

    '''
    #Plot graph
    ax.plot(x, y, marker, color=color, label=label or title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    '''

results = load_results('simulation_results_basic_system.pkl')

results = load_results('simulation_results_basic_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity', 'green_durations']}

# Basic plots
fig1, axes1 = plt.subplots(1, 1, figsize=(15, 10))
plot_graph(axes1, data['green_durations'], data['local_flow'], 'Green Duration (s)', 'Flow (cars/hour)', 'blue')

plt.savefig('durations_plot.png')

# Visualisation
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 10, 5)