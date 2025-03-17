#!/bin/bash

# Load required kernel modules
modprobe slcan
modprobe can

# Setup SLCAN device
slcand -o -s6 -t hw -S 500000 /dev/ttyUSB0 slcan0
ip link set up slcan0
ip link set slcan0 type can bitrate 500000 