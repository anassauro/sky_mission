#!/bin/bash

git clone https://github.com/raspberrypi/userland.git
cd userland
./buildme
touch ~/.bash_aliases
echo -e 'PATH=$PATH:/opt/vc/bin\nexport PATH' >> ~/.bash_aliases
echo -e 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib\nexport LD_LIBRARY_PATH' >> ~/.bash_aliases
source ~/.bashrc
sudo ldconfig
sudo reboot now