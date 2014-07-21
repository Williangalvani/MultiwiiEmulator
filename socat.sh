#!/bin/bash
USER2=$USER
sudo socat PTY,link=/dev/ttyUSB10 PTY,link=/dev/ttyUSB11&
sleep 1
sudo chown $USER2:dialout /dev/ttyUSB10
sudo chown $USER2:dialout /dev/ttyUSB11
echo "user was $USER2"
