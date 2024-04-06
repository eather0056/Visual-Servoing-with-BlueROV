import numpy as np
import matplotlib.pyplot as plt

# Given values
t_final = 20  # seconds
z_init = 0    # meters
z_final = 0.5 # meters

# Time array
t = np.linspace(0, 25, num=500)  # from 0 to 25 seconds for a smooth curve

# Coefficients for the cubic polynomial
a2 = 3 * (z_final - z_init) / (t_final**2)
a3 = -2 * (z_final - z_init) / (t_final**3)

# Calculate the desired trajectory and its derivative
z_desired = np.piecewise(t, [t < t_final, t >= t_final],
                         [lambda t: z_init + a2 * t**2 + a3 * t**3,
                          lambda t: z_final])
z_dot_desired = np.piecewise(t, [t < t_final, t >= t_final],
                               [lambda t: 2 * a2 * t + 3 * a3 * t**2,
                                lambda t: 0])

# Plotting the desired trajectories
plt.figure(figsize=(10, 5))
plt.plot(t, z_desired, label='z_desired(t)')
plt.plot(t, z_dot_desired, label='z_dot_desired(t)', linestyle='--')
plt.title('Desired Trajectories for ROV')
plt.xlabel('Time (s)')
plt.ylabel('Depth (m) / Depth Rate (m/s)')
plt.legend()
plt.grid(True)
plt.show()
