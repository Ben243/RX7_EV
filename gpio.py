#!/usr/bin/env python

# Python Implementation of ignition functionality

import RPi.GPIO as GPIO
import subprocess
from gpiozero import LED, Button
from time import sleep

#cell volts, cell temps, 12 v battery voltage, errmsg airtemp

GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP)


GPIO.wait_for_edge(3, GPIO.FALLING)

subprocess.call(['shutdown', '-h', 'now'], shell=False)