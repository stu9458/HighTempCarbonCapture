#!/bin/bash
chmod 755 /home/pi/*
pkill -9 -f heat1
python3 /home/pi/heat1.py
