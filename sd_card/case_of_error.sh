#!/bin/bash

raspistill -o test.jpg

# Remember to add toi /boot/firmware/config.txt the following lines
# start_x=1
# gpu_mem=128

# If it throws failed to open vchiq instance, do:

sudo chmod 777 /dev/vchiq