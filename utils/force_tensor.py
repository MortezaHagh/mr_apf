import sys
import numpy as np
import matplotlib.pyplot as plt

# sys.path.append("/home/piotr/mori_ws/src/mr_apf/scripts")
# from create_model import CreateModel

# --------------------------------------------------------------------------------

# Define the function f(x, y)
def f(x, y):
    return np.sin(x) + np.cos(y)


# Define the range of x and y values
x_min, x_max = -5, 5
y_min, y_max = -5, 5
x_step, y_step = 0.5, 0.5

# Create the grid of x and y values
x, y = np.meshgrid(np.arange(x_min, x_max + x_step, x_step),
                   np.arange(y_min, y_max + y_step, y_step))

# Compute the force components (Fx and Fy) based on the partial derivatives of f(x, y)
Fx = -np.gradient(f(x, y), axis=1)
Fy = -np.gradient(f(x, y), axis=0)

# Compute the magnitude of the force vectors
force_magnitude = np.sqrt(Fx**2 + Fy**2)

# Create the force tensor by combining the force components (Fx and Fy) into a 2x2 matrix
force_tensor = np.stack((Fx, Fy), axis=-1)

# Normalize the force vectors to have a constant length of 0.5
normalized_force_tensor = force_tensor / force_magnitude[..., None] * 0.5

# Plot the force vectors using the quiver function with color based on the magnitude
plt.quiver(x,
           y,
           normalized_force_tensor[..., 0],
           normalized_force_tensor[..., 1],
           force_magnitude,
           cmap="coolwarm")
plt.colorbar()
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Force Tensor Illustration")
plt.show()
