import matplotlib.pyplot as plt
from IDM import IDM as IDM
from MOBIL import MOBIL as MOBIL
from Vehicle import Vehicle as Vehicle
import numpy as np
# Initialize IDM and MOBIL models
idm = IDM()
mobil = MOBIL(politeness=0, a_threshold=0.05)

# Initialize vehicles
vehicles = [Vehicle(position=i * 10, velocity=25 + np.random.randn(), lane=np.random.choice([0, 1])) for i in range(10)]

# Simulation parameters
dt = 0.1  # Time step (s)
T = 50    # Simulation time (s)
steps = int(T / dt)

# Run simulation
positions = {0: [], 1: []}

for _ in range(steps):
    lane_0 = [v for v in vehicles if v.lane == 0]
    lane_1 = [v for v in vehicles if v.lane == 1]

    # Sort by position
    lane_0.sort(key=lambda v: v.s)
    lane_1.sort(key=lambda v: v.s)

    # Update vehicle motion
    for vehicle in vehicles:
        leader = next((v for v in (lane_0 if vehicle.lane == 0 else lane_1) if v.s > vehicle.s), None)
        vehicle.update(dt, idm, leader)

    # Decide lane changes
    for vehicle in vehicles:
        target_lane = 1 if vehicle.lane == 0 else 0
        target_vehicles = lane_1 if target_lane == 1 else lane_0
        if mobil.lane_change_decision(vehicle, target_vehicles, idm):
            vehicle.lane = target_lane  # Change lane

    # Store positions for visualization
    positions[0].append([v.s for v in lane_0])
    positions[1].append([v.s for v in lane_1])

# Visualization
plt.figure(figsize=(10, 6))
for lane, pos in positions.items():
    for i, frame in enumerate(pos):
        plt.scatter(frame, [lane] * len(frame), s=5, color='red' if lane == 0 else 'blue', alpha=0.5)
plt.xlabel("Position (m)")
plt.ylabel("Lane")
plt.title("Two-Lane Traffic Simulation with IDM and MOBIL")
plt.show()
