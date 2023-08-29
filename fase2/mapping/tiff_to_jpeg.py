from PIL import Image
import os

input_folder = "/home/helena/26AGOSTO2023/maps"
output_folder = "/home/helena/26AGOSTO2023/entregaveis"

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Loop through each image file in the input folder
for filename in os.listdir(input_folder):
    if filename.endswith(".tiff") or filename.endswith(".tif"):
        # Open the TIFF image
        image_path = os.path.join(input_folder, filename)
        tiff_image = Image.open(image_path)
        
        # Convert to RGB mode if it's RGBA
        if tiff_image.mode == 'RGBA':
            tiff_image = tiff_image.convert('RGB')
        
        # Convert and save as JPG
        output_filename = os.path.splitext(filename)[0] + ".jpg"
        output_path = os.path.join(output_folder, output_filename)
        tiff_image.save(output_path, "JPEG")
        
        print(f"Converted {filename} to {output_filename}")

print("Conversion complete!")