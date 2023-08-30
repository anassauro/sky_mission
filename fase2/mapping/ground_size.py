'''
Use this to calculate the GSD for the survey and replace this parameter in test_square.py
'''
import numpy as np

fx = 6 # Focal length in pixels (x-axis)
sensor_width = 6.287  # Sensor width in mm

# Image dimensions (in pixels)
image_width = 4056

# Drone altitude (in meters)
altitude = 6000

# Calculate GSD
gsd = (sensor_width * altitude) / (image_width * fx)

# Print the calculated GSD
print(f"Calculated GSD: {gsd:.4f} cm/pixel")

