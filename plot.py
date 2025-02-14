import matplotlib as plt 
   
fig, axs = plt.subplots(3, 2, figsize=(10, 10))
# Plot global flow vs. global density
axs[0, 0].plot(global_density, global_flow, 'bo-', label='Global Flow')
axs[0, 0].set_xlabel('Density (cars/km)')
axs[0, 0].set_ylabel('Flow (cars/hour)')
axs[0, 0].legend()
axs[0, 0].grid()

# Plot local flow vs. local density
axs[0, 1].plot(local_density, local_flow, 'ro-', label='Local Flow')
axs[0, 1].set_xlabel('Density (cars/km)')
axs[0, 1].set_ylabel('Flow (cars/hour)')
axs[0, 1].legend()
axs[0, 1].grid()

# Plot global flow vs. global average velocity
axs[1, 0].plot(global_flow, global_average_velocity, 'bo-', label='Global Flow')
axs[1, 0].set_xlabel('Flow (cars/hour)')
axs[1, 0].set_ylabel('Average Velocity (m/s)')
axs[1, 0].legend()
axs[1, 0].grid()

# Plot global flow vs. global average velocity
axs[1, 1].plot(local_flow, local_average_velocity, 'ro-', label='Local Flow')
axs[1, 1].set_xlabel('Flow (cars/hour)')
axs[1, 1].set_ylabel('Average Velocity (m/s)')
axs[1, 1].legend()
axs[1, 1].grid()

# Plot average velocity vs. density (global)
axs[2, 0].plot(global_density, global_average_velocity, 'bo-', label='Global Average Velocity')
axs[2, 0].set_xlabel('Density (cars/km)')
axs[2, 0].set_ylabel('Average Velocity (m/s)')
axs[2, 0].legend()
axs[2, 0].grid()

# Plot average velocity vs. density (local)
axs[2, 1].plot(local_density, local_average_velocity, 'ro-', label='Local Average Velocity')
axs[2, 1].set_xlabel('Density (cars/km)')
axs[2, 1].set_ylabel('Average Velocity (m/s)')
axs[2, 1].legend()
axs[2, 1].grid()

plt.tight_layout()
plt.show()