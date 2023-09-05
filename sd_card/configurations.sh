#!/bin/bash
wget -p -O ./raspi-config_20211019_all.deb https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20230214_all.deb
sudo apt -y install libnewt0.52 whiptail parted triggerhappy lua5.1 alsa-utils
sudo apt install -fy
sudo dpkg -i ./raspi-config_20230214_all.deb