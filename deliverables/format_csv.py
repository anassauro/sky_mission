import csv
import gps_time as gp

# Define the input and output file names
input_file = "teste.csv"
output_file = "output.csv"

# Open the input CSV file for reading and the output CSV file for writing
with open(input_file, mode='r') as input_csv, open(output_file, mode='w', newline='') as output_csv:
    reader = csv.reader(input_csv)
    writer = csv.writer(output_csv)

    # Read the header row
    header = next(reader)

    # Write the updated header row to the output CSV file
    header = ["UTC Time (Format YYYY-MM-DDTHH:MM:SS.sss)", "Decimal Latitude", "Decimal Longitude", "Altitude above Start Point", "Current (A)", "Voltage (V)"]
    writer.writerow(header)

    # Process each row in the CSV file
    for row in reader:
        # Extract values from the row
        gps_week_number = row[0]
        gps_time_of_week_us = row[1]
        decimal_latitude = row[2]
        decimal_longitude = row[3]
        altitude_above_start = row[4]
        current = row[7]
        voltage = row[6]

        # Perform conversions if needed
        try:
            gps_week_number = int(gps_week_number)
            gps_time_of_week_us = int(gps_time_of_week_us)
            gps_time_of_week_s = gps_time_of_week_us / 1_000_000

            gps_time1 = gp.GPSTime(gps_week_number, gps_time_of_week_s)
            utc_time = gps_time1.to_datetime().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]  # Format UTC time with milliseconds
        except ValueError as e:
            print(f"Error converting values: {e}")
            continue  # Skip to the next row if conversion fails

        # Format values as needed
        updated_row = [utc_time, decimal_latitude, decimal_longitude, altitude_above_start, current, voltage]

        # Write the updated row to the output CSV file
        writer.writerow(updated_row)

        # Print GPS Time and Datetime
        print(f"GPS Time: {gps_time1}")
        print(f"Datetime: {gps_time1.to_datetime()}")
        print("")

        # Check for lost precision
        lost_precision = gps_time1 - gp.GPSTime.from_datetime(gps_time1.to_datetime())
        print(f"Lost Precision: {lost_precision}")

print("Conversion completed. Data saved to", output_file)
