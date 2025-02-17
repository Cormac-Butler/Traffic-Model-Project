class Vehicle:
    def __init__(self, position, velocity, lane):
        self.s = position  # Position (m)
        self.v = velocity  # Speed (m/s)
        self.lane = lane  # Lane (0 or 1)
        self.dv = 0  # Speed difference with leading vehicle

    def update(self, dt, idm_model, leader):
        """
        Updates vehicle position and speed based on IDM.
        :param dt: Time step (s)
        :param idm_model: IDM instance
        :param leader: Leading vehicle
        """
        s = leader.s - self.s if leader else 1000  # Gap to leader
        dv = self.v - leader.v if leader else 0    # Relative speed

        a = idm_model.acceleration(self.v, s, dv)
        self.v += a * dt
        self.s += self.v * dt
        self.dv = dv  # Store speed difference for MOBIL
