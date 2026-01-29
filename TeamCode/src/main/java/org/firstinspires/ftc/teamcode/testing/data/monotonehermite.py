import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import PchipInterpolator

# Given data points
x = np.array([0.0, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0])
y = np.array([0.0, 90, 180, 257, 340, 423, 660])

# Create monotone Hermite (PCHIP) interpolator
pchip = PchipInterpolator(x, y)

# Generate smooth curve
x_dense = np.linspace(0, 1, 200)
y_dense = pchip(x_dense)

# Plot
plt.figure(figsize=(7, 4))
plt.plot(x_dense, y_dense, label="Monotone Hermite (PCHIP)", color='blue')
plt.scatter(x, y, color='red', label="Data points", zorder=5)
plt.title("Monotone Hermite Interpolation")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()
