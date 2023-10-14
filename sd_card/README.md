## Configuring PiCamera and MAVROS in Ubuntu 20.04
In the competition, we had to format several SD cards with similar settings. The biggest problem was the compatibility of the RaspBerry PiCamera. This camera has some specific settings that needed to be added. In addition, for the card's general functionality, an image of it was created and is in the team's SharePoint. However, it is desired that a complete script be created to run on SD cards for Outdoor competitions that has everything necessary, from MAVROS, Dronekit, OpenCV, etc.
# configurations.sh
This script will allow the Raspberry Pi to run raspi-config. 
# picamera.sh
Run this script to install configurations to use the Picamera with the commands like ```raspistill```.
# ros.sh
This script installs ROS and MAVROS tools to work with Ardupilot.
# case_of_error.sh
Run in case it shows any error when using Picamera commands on terminal.
# parameters.sh
Test PiCamera commands.
