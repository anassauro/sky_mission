#!/bin/bash

# Define the range of ISO values and shutter speeds you want to test
iso_values=(100 200 400 800)
shutter_speeds=(1000 2000 4000 8000 25000)

# Define the output directory
output_dir="/path/to/output/directory"

# Define the metering mode
metering_mode="average"

# Loop through ISO and shutter speed combinations
for iso in "${iso_values[@]}"; do
  for shutter_speed in "${shutter_speeds[@]}"; do
    # Generate the raspistill command with the current ISO, shutter speed, and metering mode
    command="raspistill -ISO $iso -ss $shutter_speed -mm $metering_mode -o $output_dir/image_ISO${iso}_SS${shutter_speed}_MM${metering_mode}.jpg"

    # Execute the command
    echo "Capturing with ISO $iso, Shutter Speed $shutter_speed, and Metering Mode $metering_mode"
    $command

    # Wait for 2 seconds before capturing the next image
    sleep 2
  done
done

echo "Capture series completed."
