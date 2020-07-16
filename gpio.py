#!/usr/bin/env python
'''
Python Implementation of GPIO functionality for the RX7 Raspberry Pi 4 module
''' 

import RPi.GPIO as GPIO
import subprocess
from gpiozero import LED, Button
from time import sleep

# inputs
GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Pi button
GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Ignition 
GPIO.setup(4 GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # ac switch
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # ac over pressure switch
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # dimmer switch

# outputs
GPIO.setup(21, GPIO.OUT) # charger relay
GPIO.setup(20, GPIO.OUT) # dc-dc relay
GPIO.setup(26, GPIO.OUT) # pump relay
GPIO.setup(16, GPIO.OUT) # ac comp relay

# pwm (max resolution 500Hz)
ac_comp_pwm = GPIO.PWM(12, 200) # ac comp pwm initialized at 200Hz
chiller_pwm = GPIO.PWM(13, 500) # chiller pwm initialized at 500Hz
cabin_pwm = GPIO.PWM(18, 500) # cabin pwm initialized at 500Hz
fan_pwm = GPIO.PWM(19, 500) # fan pwm initialized at 500Hz

# callback functions
def shutdown_seq(channel) # does the shut down
    print("shutting down...")
    subprocess.call(['shutdown', '-h', 'now'], shell=False)

def charge(channel) # turns on pumps and charger
    print('charging...')
    GPIO.output(21, 1) # charger on
    GPIO.output(26, 1) # charger on

#if falling voltage is detected on pin 3, shut down
GPIO.add_event_detect(3, GPIO.FALLING, callback=shutdown_seq)
#if falling voltage is detected on pin 4, charge
GPIO.add_event_detect(4, GPIO.FALLING, callback=charge)


# cell volts, 12 v battery voltage, cell temps, errmsg airtemp