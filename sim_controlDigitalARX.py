# Digital Control Simulation — ARX Model + Saturation (Python)
import numpy as np
import matplotlib.pyplot as plt

# -----------------------
# Sampling period
# -----------------------
Ts = 0.004  # 4 ms

# -----------------------
# Reference signal
# -----------------------
t = np.arange(0, 2 + Ts, Ts)              # 2 seconds simulation (include endpoint like MATLAB 0:Ts:2)
Ref = 5 * np.ones_like(t)                 # Step reference

# -----------------------
# ARX model coefficients
# A(z) = 1 - 1.937 z^-1 + 1.152 z^-2 - 0.2144 z^-3
# B(z) = -0.001961 z^-3
# -----------------------
a = np.array([1, -1.9366, 1.1523, -0.2144], dtype=float)  # note the sign for implementation
b = np.array([0, 0, 0, -0.001961], dtype=float)           # B(z) with zeros for delays

# -----------------------
# Initialize variables
# -----------------------
y = np.zeros_like(t, dtype=float)

y1 = 0.0
y2 = 0.0
y3 = 0.0

u1 = 0.0
u2 = 0.0
u3 = 0.0
u4 = 0.0

error1 = 0.0
Usim = np.zeros_like(t, dtype=float)

# -----------------------
# PI controller parameters
# -----------------------
Kp = -11.6083  # proportional gain
Ti = 0.1014    # integral time

# Discretization (incremental form constants)
K0 = Kp + Kp * Ts / (2 * Ti)
K1 = -Kp + Kp * Ts / (2 * Ti)

# -----------------------
# Saturation limits
# -----------------------
Umax = 100.0
Umin = -100.0

# -----------------------
# Simulation loop
# -----------------------
for k in range(len(t)):
    # ARX model
    y[k] = (-a[1] * y1
            -a[2] * y2
            -a[3] * y3
            + b[0] * u1
            + b[1] * u2
            + b[2] * u3
            + b[3] * u4)

    # Control error
    error = Ref[k] - y[k]

    # PI control (incremental form)
    u = u1 + K0 * error + K1 * error1

    # Saturation
    if u > Umax:
        u = Umax
    elif u < Umin:
        u = Umin

    # Update past values
    y3, y2, y1 = y2, y1, y[k]
    u4, u3, u2, u1 = u3, u2, u1, u
    error1 = error
    Usim[k] = u

# -----------------------
# Plot results
# -----------------------
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, y, 'b+', markersize=4, label='System Output')
plt.plot(t, Ref, 'r--', label='Reference')
plt.xlabel('Time (s)')
plt.ylabel('Output y(k)')
plt.legend()
plt.title('ARX System Response')

plt.subplot(2, 1, 2)
plt.plot(t, Usim, 'k+', markersize=4)
plt.xlabel('Time (s)')
plt.ylabel('Control Signal u(k)')
plt.title('PI Control Signal with Saturation')

plt.tight_layout()
plt.show()
