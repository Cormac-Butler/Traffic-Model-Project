import matplotlib.pyplot as plt
import statsmodels.api as sm
import TrafficVisualization as tv
import pickle

def load_results(filename):

    with open(filename, 'rb') as file:
        return pickle.load(file)

def plot_scatter(ax, x, y, xlabel, ylabel, title, color, marker='o-', label=None):

    ax.plot(x, y, marker, color=color, label=label or title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid()

def plot_best_fit_lowess(ax, x, y, xlabel, ylabel, title, color):

    ax.scatter(x, y, color='black', marker='x', label='Data Points', alpha=0.25)
    lowess = sm.nonparametric.lowess(y, x)
    ax.plot(lowess[:, 0], lowess[:, 1], color=color, linestyle='-', label='LOWESS Fit')
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid()

results = load_results('simulation_results_traffic_lights_system.pkl')
data = {key: results.get(key, []) for key in ['cars_positions', 'road_length', 'global_density', 'global_flow', 
                                              'local_density', 'local_flow', 'global_average_velocity', 'local_average_velocity']}

# First figure: Basic plots
fig1, axes1 = plt.subplots(2, 3, figsize=(15, 10))
plot_scatter(axes1[0, 0], data['global_density'], data['global_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plot_scatter(axes1[0, 1], data['global_density'], data['global_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Global Avg Velocity vs. Density', 'blue')
plot_scatter(axes1[0, 2], data['global_flow'], data['global_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Global Flow vs. Velocity', 'blue')
plot_scatter(axes1[1, 0], data['local_density'], data['local_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Local Flow vs. Density', 'red')
plot_scatter(axes1[1, 1], data['local_density'], data['local_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Local Avg Velocity vs. Density', 'red')
plot_scatter(axes1[1, 2], data['local_flow'], data['local_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Local Flow vs. Velocity', 'red')
plt.tight_layout()
plt.savefig('graph3.png')

# Second figure: LOWESS plots
fig2, axes2 = plt.subplots(3, 2, figsize=(12, 12))
plot_best_fit_lowess(axes2[0, 0], data['global_density'], data['global_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Global Flow vs. Density', 'blue')
plot_best_fit_lowess(axes2[0, 1], data['global_flow'], data['global_average_velocity'], 'Flow (cars/hour)', 'Average Velocity (m/s)', 'Flow vs. Velocity (Global)', 'blue')
plot_best_fit_lowess(axes2[1, 0], data['local_density'], data['local_flow'], 'Density (cars/km)', 'Flow (cars/hour)', 'Local Flow vs. Density', 'red')
plot_best_fit_lowess(axes2[1, 1], data['global_density'], data['global_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Global Avg Velocity vs. Density', 'blue')
plot_best_fit_lowess(axes2[2, 0], data['local_density'], data['local_average_velocity'], 'Density (cars/km)', 'Average Velocity (m/s)', 'Local Avg Velocity vs. Density', 'red')
axes2[2, 1].axis('off')
plt.tight_layout()
plt.savefig('graph4.png')

# Visualization
tv.TrafficVisualization(data['cars_positions'][0], data['road_length'], 5, 250, 5)