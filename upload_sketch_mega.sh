#!/bin/bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 joy_subscriber
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 joy_subscriber
