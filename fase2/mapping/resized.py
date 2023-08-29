from PIL import Image
import os

input_folder = "/home/helena/26AGOSTO2023/images"
output_folder = "/home/helena/26AGOSTO2023/resized"
target_size = (640, 480)

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Loop through each image file in the input folder
for filename in os.listdir(input_folder):
    if filename.endswith(".jpg") or filename.endswith(".png"):
        # Open the image
        image_path = os.path.join(input_folder, filename)
        image = Image.open(image_path)
        
        # Resize the image
        resized_image = image.resize(target_size,  Image.Resampling.LANCZOS)
        
        # Save the resized image to the output folder
        output_path = os.path.join(output_folder, filename)
        resized_image.save(output_path)
        
        print(f"Resized {filename} to {target_size}")

print("Resizing complete!")
