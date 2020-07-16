#!/usr/bin/env python
'''
Python 3 Implementation of GPIO and serial functionality 
for the RX7 Raspberry Pi 4 module
''' 
import RPi.GPIO as GPIO
from gpiozero import LED, Button
import subprocess
import os, sys
import serial
# import can
from time import sleep

'''
SERIAL INITIALIZATION
'''
# inputs separated by spaces:
# cell volts, 12 v battery voltage, cell temps, errmsg airtemp
usb0 = serial.Serial('/dev/ttyUSB0', 9600) # usb serial input 1

# usb1 = serial.Serial('/dev/ttyUSB1', 9600) 

'''
CAN INITIALIZATION (TO BE IMPLEMENTED) 
'''
# # stuff below is to initialize the can module to receive messages
# os.system('sudo ip link set can0 type can bitrate 100000')
# os.system('sudo ifconfig can0 up')

# can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')# socketcan_native

# #msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], extended_id=False)
# msg = can0.recv(10.0)
# print msg
# if msg is None:
#     print('Timeout occurred, no message.')

# os.system('sudo ifconfig can0 down')

'''
GPIO INITIALIZATION
'''
# inputs
GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Pi power button
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

'''event detects'''

#if falling voltage is detected on pin 3, shut down
GPIO.add_event_detect(3, GPIO.FALLING, callback=shutdown_seq)
# #if falling voltage is detected on pin 4, charge
# GPIO.add_event_detect(4, GPIO.FALLING, callback=charge)


''' variable name declarations '''
ign_switch = False
pi_button = False
ac_switch = False
acOver


'''error declarations'''
# All errors are just booleans
acOverPressureError = False
battVoltError = False
battTempError = False
motorInverterTempError = False
battTempDevError = False
battVoltDevError = False

# error calls
def check_errors(): # prints any errors and returns whether any are true
    err_string = ''
    if acOverPressureError: err_string += 'acOverPressureError '
    if battVoltError: err_string += 'battVoltError '
    if battTempError: err_string += 'battTempError ' 
    if motorInverterTempError: err_string += 'motorInverterTempError ' 
    if battTempDevError: err_string += 'battTempDevError '
    if battVoltDevError: err_string += 'battVoltDevError '
    print(err_string)
    return (acOverPressureError or battVoltError or battTempError 
            or motorInverterTempError or battTempDevError or battVoltDevError)

def update_errors():
    return

while True:
    sleep(0.01)



    
