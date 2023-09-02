from sklearn.cluster import KMeans
import numpy as np
from collections import defaultdict

# Simulated coordinates
coordinates = [
    (50.90756, 6.22439), # obj 1
    (50.90758, 6.22465), # obj 1
    (50.90736, 6.2244), # obj 1
    (50.90873, 6.22478), # obj 3
    (50.90737, 6.22459), # obj 1
    (50.90813, 6.2257), # obj 2
    (50.90871, 6.22459), # obj 3
    (50.9087, 6.22442), # obj 3
    (50.90881, 6.22454), # obj 3
    (50.90811, 6.22548), # obj 2
    (50.90797, 6.22545), # obj 2
    (50.90882, 6.22471), # obj 3
    (50.90798, 6.22568), # obj 2
    # ... more coordinates
]

# Convert coordinates to radians for distance calculations
coordinates_rad = np.radians(coordinates)

# KMeans parameters (number of clusters)
n_clusters = 3  # You can adjust this based on your expectations

# Perform KMeans clustering
kmeans = KMeans(n_clusters=n_clusters)
clusters = kmeans.fit_predict(coordinates_rad)

# Organize coordinates into clusters
cluster_to_coordinates = defaultdict(list)
for (lat, lon), cluster in zip(coordinates, clusters):
    cluster_to_coordinates[cluster].append((lat, lon))

# Print clustered coordinates
for cluster, coord_list in cluster_to_coordinates.items():
    print(f"Quadrado {cluster+1}:")
    for lat, lon in coord_list:
        print(f"Coordinates: ({lat}, {lon})")
    print("---------")
