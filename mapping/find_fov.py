import numpy as np

# Prepare
w, h = 1280, 720
fx, fy = 629.60088304, 628.99975883

# Go
fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))

print("Field of View (degrees):")
print(f"  {fov_x = :.5f}\N{DEGREE SIGN}")
print(f"  {fov_y = :.5f}\N{DEGREE SIGN}")