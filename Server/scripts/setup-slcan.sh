#!/bin/zsh

# Load required kernel modules
modprobe slcan
modprobe can

# Setup SLCAN device
slcand -c -o -f -s8 /dev/ttyACM0 can0
ip link set up can0
ip link set can0 type can bitrate 500000
