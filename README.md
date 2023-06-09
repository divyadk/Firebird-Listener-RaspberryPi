# Listener for Firebird-ATMEGA2560 - Communicating with Raspberry Pi 

## Why this listener?
The ATMEGA2560 on-board the Firebird V has limited write cycles (around 10,000 as per NEX Robotics). In this project, I have a listener file that is to be run on the ATMEGA2560. With serial communication from Raspberry Pi on board the robot with the ATMEGA2560, seamless communication between these both can be achieved.

## Steps

1. Write the main.c file on your Firebird
2. Mount the Raspberry Pi on top of your Firebird and connect it to the Firebird with the USB Cable provided (Xbee also works)
3. Write the file move.py on your Raspberry Pi

With move.py, the sensor values can be fetched and the robot can be moved by specifying the required left and right motor's velocity values

(Refer the main.c file for more details)