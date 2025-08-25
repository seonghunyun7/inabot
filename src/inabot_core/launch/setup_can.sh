#!/bin/bash
set -e

INTERFACE=can0
BITRATE=1000000

echo "[INFO] Setting up $INTERFACE with bitrate $BITRATE"

sudo /usr/sbin/ip link set $INTERFACE down
sudo /usr/sbin/ip link set $INTERFACE type can bitrate $BITRATE
sudo /usr/sbin/ip link set $INTERFACE up

echo "[INFO] $INTERFACE setup complete."

