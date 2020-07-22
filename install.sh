#!/bin/bash

echo "##############################################"
echo "Installing Bricks V2 IMU drivers..."
echo "##############################################"

# bricks viewer (launch via brickv) to get the UID of each unit
sudo apt-get install python python-qt4 python-qt4-gl python-opengl python-serial
sudo wget http://download.tinkerforge.com/tools/brickv/linux/brickv_linux_latest.deb
sudo dpkg -i brickv_linux_latest.deb

# get the bricks daemon
sudo wget http://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_amd64.deb
sudo dpkg -i brickd_linux_latest_amd64.deb

# start bricks daemon
sudo systemctl start brickd-resume.service

echo "##############################################"
echo "Finished installing Bricks V2 IMU drivers"
echo "##############################################"

