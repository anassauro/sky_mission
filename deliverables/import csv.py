import csv
import gps_time as gp

# Define the input and output file names
input_file = "teste.csv"
output_file = "output.csv"

# Open the input CSV file for reading and the output CSV file for writing
with open(input_file, mode='r') as input_csv, open(output_file, mode='w', newline='') as output_csv:
    reader = csv.reader(input_csv)
    writer = csv.writer(output_csv)

    # Read and skip the header row
    header = next(reader)

    # Write the updated header row to the output CSV file
    header = ["UTC Time (Format YYYY-MM-DDTHH:MM:SS.sss)", "Decimal Latitude", "Decimal Longitude", "Altitude above Start Point"]
    writer.writerow(header)

    for row in reader:
        # Parse the GPS week number and GPS time of the week (in microseconds) from the input CSV
        gps_week_number = int(row[0])
        gps_time_of_week_us = int(row[1])

        # Convert microseconds to seconds
        gps_time_of_week_s = gps_time_of_week_us / 1_000_000_000

        if((gps_time_of_week_s/100000) < 1 and gps_time_of_week_s > 0.1):
            gps_time_of_week_s = gps_time_of_week_s * 10


        # Calculate the GPSTime based on the GPS week number and GPS time of the week in seconds
        gps_time1 = gp.GPSTime(gps_week_number, gps_time_of_week_s)

        # Replace the GPS week and time columns with the GPSTime converted to a datetime object
        row[0] = gps_time1.to_datetime().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]  # Format UTC time
        row[1] = f"{float(row[2])}"  # Format Decimal Latitude
        row[2] = f"{float(row[3])}"  # Format Decimal Longitude
        row[3] = ""  # Format Altitude above Start Point
        # Replace the GPS week and time columns with the GPSTime converted to a datetime object
        # row[0] = gps_time1.to_datetime().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]  # Format UTC time
        # row[1] = row[2]  # Format Decimal Latitude
        # row[2] = row[3]  # Format Decimal Longitude
        # row[3] = ""

        # Write the updated row to the output CSV file
        writer.writerow(row)

        # Print GPS Time and Datetime
        print(f"GPS Time: {gps_time1}")
        print(f"Datetime: {gps_time1.to_datetime()}")
        print("")

        # Check for lost precision
        lost_precision = gps_time1 - gp.GPSTime.from_datetime(gps_time1.to_datetime())
        print(f"Lost Precision: {lost_precision}")

print("Conversion completed. Data saved to", output_file)
