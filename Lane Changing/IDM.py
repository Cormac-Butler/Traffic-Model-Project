import numpy as np

class IDM:
    def __init__(self, v0=30, T=1.5, a=1.0, b=2.0, delta=4, s0=2.0):
        self.v0 = v0  # Desired speed (m/s)
        self.T = T    # Safe time headway (s)
        self.a = a    # Maximum acceleration (m/s^2)
        self.b = b    # Comfortable deceleration (m/s^2)
        self.delta = delta  # Acceleration exponent
        self.s0 = s0  # Minimum gap (m)

    def acceleration(self, v, s, dv):
        """
        Computes the acceleration of a vehicle based on IDM.
        :param v: Current speed (m/s)
        :param s: Gap to the leading vehicle (m)
        :param dv: Speed difference (v - v_lead) (m/s)
        :return: Acceleration (m/s^2)
        """
        s_star = self.s0 + v * self.T + (v * dv) / (2 * np.sqrt(self.a * self.b))
        return self.a * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2) if s > 0 else -self.b
