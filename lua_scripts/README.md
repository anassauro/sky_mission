# Lua scripts

At IMAV, we encountered several issues with telemetry interference. This ended up ruining our testing day. We were unable to connect with the drone through the telemetry connected to the computer. 
This made it impossible to start missions and put the ability to obtain a complete flight log at risk. This is because in the middle of the flight there was a risk of momentarily losing the connection and thus, also the information about the route taken.
Therefore, we decided to run two other codes within the controller, in addition to the geofence. These are described below.

## log.lua

The log.lua file must be added to the controller's SD card. From the moment the vehicle is armed, GPS and time information is written to the CSV file. 
To standardize the GPS time scale to GMT, you must use the import csv.py code.

## mission.lua

The mission.lua code allows the start of the mission that was uploaded to the controller when the drone is deployed. It automatically takesoff the drone and when the takeoff position is reached, it starts the mission. This allowed us to depend solely on the RC connection to start the mission.

