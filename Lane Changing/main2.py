import numpy as np
import matplotlib.pyplot as plt

# Constants
NUM_LANES = 2  # Number of lanes
ROAD_LENGTH = 1000  # Length of the road (meters)
MAX_SPEED = 30  # Maximum speed (m/s)
TIME_STEP = 0.1  # Time step (seconds)
SIM_TIME = 60  # Total simulation time (seconds)
NUM_VEHICLES = 10  # Number of vehicles
PLOT_INTERVAL = 10  # Plot every N steps

# IDM Parameters
DESIRED_SPEED = 30  # Desired speed (m/s)
TIME_HEADWAY = 1.5  # Safe time headway (s)
MIN_GAP = 2  # Minimum gap (m)
MAX_ACCEL = 1.5  # Maximum acceleration (m/s²)
COMFORT_DECEL = 1.5  # Comfortable deceleration (m/s²)

# MOBIL Parameters
POLITENESS = 0.1  # Politeness factor (0 = selfish, 1 = polite)
SAFETY_DECEL = 2.0  # Maximum safe deceleration for lane change (m/s²)
BIAS_RIGHT = 0.1  # Bias for changing to the right lane

# Vehicle class
class Vehicle:
    def __init__(self, id, lane, pos, vel):
        self.id = id
        self.lane = lane
        self.pos = pos
        self.vel = vel
        self.acc = 0

    def update(self, acc, time_step):
        self.vel += acc * time_step
        self.vel = max(0, min(self.vel, MAX_SPEED))  # Ensure velocity is within bounds
        self.pos += self.vel * time_step
        self.pos %= ROAD_LENGTH  # Wrap around the road

# IDM acceleration function
def idm_acceleration(vehicle, front_vehicle):
    if front_vehicle is None:
        return MAX_ACCEL * (1 - (vehicle.vel / DESIRED_SPEED) ** 4)

    delta_v = vehicle.vel - front_vehicle.vel
    s = front_vehicle.pos - vehicle.pos - 5  # 5m is the length of the vehicle
    s_star = MIN_GAP + max(0, vehicle.vel * TIME_HEADWAY + (vehicle.vel * delta_v) / (2 * np.sqrt(MAX_ACCEL * COMFORT_DECEL)))
    acc = MAX_ACCEL * (1 - (vehicle.vel / DESIRED_SPEED) ** 4 - (s_star / s) ** 2)
    return acc

# MOBIL lane change decision
def mobil_lane_change(vehicle, front_vehicle, front_target, rear_target):
    if front_vehicle is None:
        return False

    # Acceleration in current lane
    acc_current = idm_acceleration(vehicle, front_vehicle)

    # Acceleration in target lane
    acc_target = idm_acceleration(vehicle, front_target)

    # Calculate rear vehicle's acceleration in current lane
    if rear_target is not None:
        rear_front_vehicle = None
        min_dist = float('inf')
        for other in vehicles:
            if other.lane == rear_target.lane and other.pos > rear_target.pos and (other.pos - rear_target.pos) < min_dist:
                rear_front_vehicle = other
                min_dist = other.pos - rear_target.pos
        acc_rear_current = idm_acceleration(rear_target, rear_front_vehicle)
    else:
        acc_rear_current = 0  # No rear vehicle in the target lane

    # Calculate rear vehicle's acceleration in target lane after lane change
    if rear_target is not None:
        acc_rear_target = idm_acceleration(rear_target, vehicle)
    else:
        acc_rear_target = 0  # No rear vehicle in the target lane

    # Check if lane change is beneficial
    if acc_target - acc_current < POLITENESS * (acc_rear_target - acc_rear_current):
        return False

    # Check safety criterion
    if rear_target is not None and acc_rear_target < -SAFETY_DECEL:
        return False

    return True

# Initialize vehicles
vehicles = [Vehicle(i, np.random.randint(0, NUM_LANES), np.random.uniform(0, ROAD_LENGTH), np.random.uniform(0, MAX_SPEED)) for i in range(NUM_VEHICLES)]

# Simulation loop
for step in range(int(SIM_TIME / TIME_STEP)):
    # Update vehicle states
    for vehicle in vehicles:
        # Find front vehicle in the same lane
        front_vehicle = None
        min_dist = float('inf')
        for other in vehicles:
            if other.lane == vehicle.lane and other.pos > vehicle.pos and (other.pos - vehicle.pos) < min_dist:
                front_vehicle = other
                min_dist = other.pos - vehicle.pos

        # Calculate acceleration using IDM
        acc = idm_acceleration(vehicle, front_vehicle)
        vehicle.update(acc, TIME_STEP)

        # Lane change using MOBIL
        if np.random.rand() < 0.1:  # Random chance to consider lane change
            target_lane = (vehicle.lane + 1) % NUM_LANES  # Try changing to the next lane
            front_target = None
            rear_target = None
            min_dist_front = float('inf')
            min_dist_rear = float('inf')
            for other in vehicles:
                if other.lane == target_lane:
                    if other.pos > vehicle.pos and (other.pos - vehicle.pos) < min_dist_front:
                        front_target = other
                        min_dist_front = other.pos - vehicle.pos
                    if other.pos < vehicle.pos and (vehicle.pos - other.pos) < min_dist_rear:
                        rear_target = other
                        min_dist_rear = vehicle.pos - other.pos

            if mobil_lane_change(vehicle, front_vehicle, front_target, rear_target):
                vehicle.lane = target_lane

    # Plot vehicles
    if step % PLOT_INTERVAL == 0:
        plt.clf()
        for vehicle in vehicles:
            plt.scatter(vehicle.pos, vehicle.lane, c='blue')
        plt.xlim(0, ROAD_LENGTH)
        plt.ylim(-0.5, NUM_LANES - 0.5)
        plt.xlabel("Position (m)")
        plt.ylabel("Lane")
        plt.title(f"Time: {step * TIME_STEP:.1f} s")
        plt.pause(0.01)

plt.show()