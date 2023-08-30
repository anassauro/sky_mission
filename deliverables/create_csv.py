'''
Creates a csv file with relevant information. 
The file format is the Telemetry CSV log file from QGroundControl
Remember to tick the 'Save CSV file' in Application Settings before the mission :)
'''

import pandas as pd

df = pd.read_csv('/home/helena/26AGOSTO2023/2023-08-29 23-46-37 vehicle1.csv') # path to qground csv log file

df = df[['Timestamp','gps.lat', 'gps.lon', 'altitudeRelative']] # columns we want

df.to_csv('/home/helena/26AGOSTO2023/log.csv', index=False) # create log file to be handled to the judges
