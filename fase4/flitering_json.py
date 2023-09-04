import json
import numpy as np
from sklearn.cluster import DBSCAN

# Load the object detection results from the JSON file
with open("object_detections.json", "r") as json_file:
    object_detections = json.load(json_file)

# Define a function to check if two bounding boxes overlap
def bbox_overlap(bb1, bb2, threshold=0.2):
    x1, y1, w1, h1 = bb1
    if bb2:
        x2, y2, w2, h2 = bb2
    else:
        x2, y2, w2, h2 = 0, 0, 0, 0

    # Calculate the overlap area
    overlap_x = max(0, min(x1 + w1, x2 + w2) - max(x1, x2))
    overlap_y = max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
    overlap_area = overlap_x * overlap_y

    # Calculate the area of the smaller bounding box
    area_bb1 = w1 * h1
    area_bb2 = w2 * h2
    min_area = min(area_bb1, area_bb2)

    # Calculate the overlap ratio
    overlap_ratio = overlap_area / min_area

    return overlap_ratio >= threshold

# Group the detections by class
grouped_detections = {}
for image_path, detections in object_detections.items():
    for detection in detections:
        class_id = detection["class"]
        if class_id not in grouped_detections:
            grouped_detections[class_id] = []
        grouped_detections[class_id].append({"image_path": image_path, "bbox": detection["coordinates"]})

# Define parameters
group_size = 10  # Number of images in each group
min_cluster_size = 3  # Minimum cluster size to be valid
output_file = "clustered_detections.json"

# Create a dictionary to store the clustered detections
clustered_detections = {}

# Create a dictionary to store the total number of tracked objects
total_tracked_objects = {}

# Iterate over the grouped detections by class
for class_id, detections in grouped_detections.items():
    # Combine all the detections from all images for this class
    all_bounding_boxes = [detection["bbox"] for detection in detections if detection["bbox"]]
    all_image_paths = [detection["image_path"] for detection in detections if detection["bbox"]]

    # Extract bounding box coordinates for DBSCAN clustering
    bounding_boxes = np.array(all_bounding_boxes)

    # Apply DBSCAN clustering with an adjusted eps parameter
    eps = 90  # Adjust this value as needed
    clustering = DBSCAN(eps=eps, min_samples=min_cluster_size, metric='euclidean')
    clustering.fit(bounding_boxes)
    cluster_labels = clustering.labels_

    # Create a dictionary to store clustered detections for this class
    clustered_detections[class_id] = []

    # Find unique clusters
    unique_labels = np.unique(cluster_labels)
    for label in unique_labels:
        if label == -1:
            continue  # Skip noise points
        cluster_indices = np.where(cluster_labels == label)[0]

        # Extract bounding boxes and image paths for the cluster
        cluster_bboxes = [all_bounding_boxes[idx] for idx in cluster_indices]
        cluster_images = [all_image_paths[idx] for idx in cluster_indices]

        # Calculate the average bounding box
        average_bbox = np.mean(cluster_bboxes, axis=0).tolist()

        # Add the clustered detection to the list
        clustered_detections[class_id].append({
            "class": class_id,
            "average_bbox": average_bbox,
            "images": cluster_images
        })

        # Update the total number of tracked objects
        if class_id not in total_tracked_objects:
            total_tracked_objects[class_id] = 0
        total_tracked_objects[class_id] += 1

# Save the clustered detections to a JSON file
with open(output_file, "w") as output_json:
    json.dump(clustered_detections, output_json, indent=4)

print(f"Clustered detections saved to {output_file}")

# Print the total number of tracked objects for each class
print("Total Number of Tracked Objects:")
for class_id, count in total_tracked_objects.items():
    print(f"Class {class_id}: {count} tracked objects")
