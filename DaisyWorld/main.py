from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt

def DE(t, y):
    a_w, a_b = y
    a_g = p - a_w - a_b

    temps = calc_temps(y, L)

    beta_w = (beta_T(temps[1] ** 0.25))
    beta_b = (beta_T(temps[2] ** 0.25))

    return [a_w * (a_g * beta_w - gamma), a_b * (a_g * beta_b - gamma)]

def beta_T(T):
     return 1 - k * (T - T_opt) ** 2 if abs(T - T_opt) < k ** (-.5) else 0
     
def calc_temps(y, L):
    a_w, a_b = y
    a_g = p - a_w - a_b

    alpha_p = a_w * alpha_w + a_b * alpha_b + a_g * alpha_g

    T4 = s_0 * L * (1 - alpha_p) / sigma

    T_w4 = (q * (alpha_p - alpha_w) + T4)
    T_b4 = (q * (alpha_p - alpha_b) + T4)
    T_g4 = (q * (alpha_p - alpha_g) + T4)

    print(T4** .25)
    return [T4, T_w4, T_b4, T_g4]


     
# Declaring Variables
sigma = 5.67 * 10 ** -8 # Boltzmann constant
s_0 = 917 # Solar irradiance
q = 2.06 * 10 ** 9 # Heat transfer coefficient
T_1 = 278 # Minimum temperature
T_2 = 313   # Maximum temperature
T_opt = T_1 + T_2 / 2 # Optimal Temperature
k = 1 / (T_2 - T_1 / 2) ** 2 # Parabolic width
alpha_g = .5 # Albedo of the ground
alpha_w = .8 # Albedo of the white daisies
alpha_b = .3 # Albedo of the black daises  
p = 1 # Fraction of land that is fertile
gamma = .3 # Death rate

a_w0 = .01 # Fraction of ground covered by white daisies
a_b0 = .01 # Fraction of ground covered by black daisies
a_g = p - a_w0 - a_b0
L = 0.9 # Luminosity 
t_span = (0, 1000)

params = (L, gamma, p)
initial = [a_w0, a_b0]
res = solve_ivp(DE, t_span, initial, dense_output = True, rtol=1e-8, atol=1e-8)

t = np.linspace(0, 1000, 300)
plt.plot(t, res.sol(t)[0], label = 'White')
plt.plot(t, res.sol(t)[1], label = 'Black')
plt.legend()
plt.show()