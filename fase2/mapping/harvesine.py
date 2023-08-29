import math

def haversine_distance(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000  
    
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    return distance

def are_points_close(lat1, lon1, lat2, lon2, threshold_meters=10):
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    return distance < threshold_meters

# Example coordinates
lat1, lon1 = -23.568200, -46.736659 
lat2, lon2 = -23.56825, -46.73660  # square

if are_points_close(lat1, lon1, lat2, lon2):
    print("The points are less than 10 meters apart.")
else:
    print("The points are more than 10 meters apart.")
