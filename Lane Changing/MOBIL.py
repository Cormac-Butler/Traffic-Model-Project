class MOBIL:
    def __init__(self, politeness=0.5, a_threshold=0.2):
        self.politeness = politeness  # Politeness factor (0 = aggressive, 1 = very polite)
        self.a_threshold = a_threshold  # Threshold for lane change (m/s^2)

    def lane_change_decision(self, vehicle, lane_vehicles, idm_model):
        """
        Determines if a vehicle should change lanes.
        :param vehicle: The vehicle considering a lane change.
        :param lane_vehicles: List of vehicles in the target lane.
        :param idm_model: IDM model for acceleration computation.
        :return: True if the vehicle should change lanes, False otherwise.
        """
        current_a = idm_model.acceleration(vehicle.v, vehicle.s, vehicle.dv)

        # Find leader and follower in the target lane
        leader, follower = self.find_neighbors(vehicle, lane_vehicles)

        # Compute new acceleration in the new lane
        new_a = idm_model.acceleration(vehicle.v, leader.s if leader else 1000, vehicle.dv)

        # Compute follower's acceleration in the target lane
        if follower:
            follower_a = idm_model.acceleration(follower.v, max(follower.s - vehicle.s, 2), follower.dv)
        else:
            follower_a = 0  # No follower, assume no braking needed

        # **More aggressive lane change conditions**
        if (new_a - current_a) > self.a_threshold and (follower_a - current_a) > -self.politeness * follower_a:
            return True  # Change lanes

        return False  # Stay in current lane



    @staticmethod
    def find_neighbors(vehicle, lane_vehicles):
        """
        Finds the leading and following vehicles in the target lane.
        """
        leader, follower = None, None
        for other in lane_vehicles:
            if other.s > vehicle.s and (leader is None or other.s < leader.s):
                leader = other
            elif other.s < vehicle.s and (follower is None or other.s > follower.s):
                follower = other
        return leader, follower
