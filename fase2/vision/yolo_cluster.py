from sklearn.cluster import KMeans
import numpy as np
from collections import defaultdict

def parse_coordinates_file(file_path):
    coordinates_data = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        lat, lon, distance = None, None, None
        for line in lines:
            if line.startswith("Latitude"):
                lat = float(line.split(":")[1].strip().split()[0])
            elif line.startswith("Longitude"):
                lon = float(line.split(":")[1].strip().split()[0])
            elif line.startswith("Distance to Center"):
                distance = float(line.split(":")[1].strip().split()[0])
                if lat is not None and lon is not None and distance is not None:
                    coordinates_data.append((lat, lon, distance))
                    lat, lon, distance = None, None, None
    return coordinates_data

# Load and parse coordinates data from the file
coordinates_data = parse_coordinates_file('/home/helena/26AGOSTO2023/coordinates.txt')


# Convert coordinates to radians for distance calculations
coordinates_rad = np.radians(coordinates_data)

# KMeans parameters (number of clusters)
n_clusters = 1  # You can adjust this based on your expectations

# Perform KMeans clustering
kmeans = KMeans(n_clusters=n_clusters)
clusters = kmeans.fit_predict(coordinates_rad)

# Perform KMeans clustering
kmeans = KMeans(n_clusters=1)

clusters = kmeans.fit_predict(coordinates_rad)

# Organize coordinates into clusters
cluster_to_coordinates = defaultdict(list)
for (lat, lon, _), cluster in zip(coordinates_data, clusters):
    cluster_to_coordinates[cluster].append((lat, lon))


# Calculate the coordinates with the lowest distance within each cluster
cluster_min_distance_coords = {}
for cluster, coord_list in cluster_to_coordinates.items():
    min_distance_coords = None
    min_distance = float('inf')
    for lat, lon in coord_list:
        for data_lat, data_lon, data_distance in coordinates_data:
            if lat == data_lat and lon == data_lon:
                if data_distance < min_distance:
                    min_distance = data_distance
                    min_distance_coords = (lat, lon)
                break
    cluster_min_distance_coords[cluster] = min_distance_coords

# Print clustered coordinates with the lowest distance
for cluster, min_distance_coords in cluster_min_distance_coords.items():
    print(f"Cluster {cluster+1}:")
    if min_distance_coords is not None:
        print(f"Coordinates with lowest distance to aproximate the square: {min_distance_coords}")
    else:
        print("No matching coordinates found.")
    print("---------")
