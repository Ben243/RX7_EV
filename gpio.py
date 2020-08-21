#!/usr/bin/env python
'''
Python 3 Implementation of GPIO and serial functionality 
for the RX7 Raspberry Pi 4 module
''' 
import RPi.GPIO as GPIO
from gpiozero import LED, Button
import subprocess
import os, sys
import threading
import serial
# import can
from time import sleep
import numpy as np

'''
SERIAL INITIALIZATION
'''
# inputs separated by spaces:
# cell volts, lv battery voltage, cell temps, errmsg airtemp
usb0 = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # usb serial input 1

# usb1 = serial.Serial('/dev/ttyUSB1', 9600, timeout=1) 

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
GPIO.setup(4 GPIO.IN, pull_up_down=GPIO.PUD_UP) # ac switch
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_UP) # ac over pressure switch
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP) # dimmer switch

# outputs
GPIO.setup(21, GPIO.OUT) # charger relay
GPIO.setup(20, GPIO.OUT) # twelve volt dc-dc relay
GPIO.setup(26, GPIO.OUT) # pump relay
GPIO.setup(16, GPIO.OUT) # ac comp relay

# pwm (software)
ac_comp_pwm = GPIO.PWM(12, 200) # ac comp pwm initialized at 200Hz.
ac_comp_pwm.start(85) # initialize ac compressor pwm at 85
# Note: duty cycle range means 5% is max, and 85% is min

chiller_pwm = GPIO.PWM(13, 1000) # chiller pwm initialized at 500Hz
chiller_pwm.start(0)
cabin_pwm = GPIO.PWM(18, 1000) # cabin pwm initialized at 500Hz
cabin_pwm.start(0)
fan_pwm = GPIO.PWM(19, 1000) # fan pwm initialized at 500Hz
fan_pwm.start(0)

# callback functions
def shutdown_seq(channel) # does the shut down
    print("shutting down...")
    subprocess.call(['shutdown', '-h', 'now'], shell=False)

def charge(channel) # turns on pumps and charger
    print('charging...')
    GPIO.output(21, 1) # charger on
    GPIO.output(26, 1) # charger on


'''event detects: for any gpio events that can be done outside of loop'''

#if falling voltage is detected on pin 3, shut down pi
GPIO.add_event_detect(3, GPIO.FALLING, callback=shutdown_seq)
# #if falling voltage is detected on pin 4, charge
# GPIO.add_event_detect(4, GPIO.FALLING, callback=charge)


''' variable name declarations '''
# All variables are initialized here
ign_switch = False
# pi_button = False #gpio events are declared outside of loop
ac_switch = False
ac_over_pressure_switch = False
dimmer_switch = False
cell_volts = np.arange(192) #[0 for x in range(192)] # 192, or 96*2 individual values
batt_temps = np.arange(16) #[0 for x in range(16)] # 16 battery temps
# errmsg = False # arduino errors, see error declarations
air_temp = 0
motor_inverter_temp = 0
morot_stator_temp = 0
high_pressure_refrig = 0
lo_pressure_refrig = 0
temp_refrig = [] # count unsure TODO CHECK for final count

'''error declarations'''
# All errors are just booleans
acOverPressureError = False
battVoltError = False
battTempError = False
motorInverterTempError = False
motorStatorTempError
battTempDevError = False
battVoltDevError = False
serialReadError = False
ArduinoError = False

# error calls
def check_errors(): # prints any errors and returns whether any are true
    err_string = ''
    if serialReadError: err_string += 'serialReadError ' #serial not working
    if acOverPressureError: err_string += 'acOverPressureError '
    if battVoltError: err_string += 'battVoltError '
    if battTempError: err_string += 'battTempError ' 
    if motorInverterTempError: err_string += 'motorInverterTempError ' # ASK KEVIN WHERE THESE COME FROM
    if motorStatorTempError: err_string += 'motorStatorTempError ' # ASK KEVIN WHERE THESE COME FROM
    if battTempDevError: err_string += 'battTempDevError '
    if battVoltDevError: err_string += 'battVoltDevError '
    if ArduinoError: err_string += 'ArduinoError '
    print(err_string)
    return (serialReadError or acOverPressureError or battVoltError or battTempError 
            or motorInverterTempError or motorStatorTempError or battTempDevError or battVoltDevError or ArduinoError)

# def update_errors(arderr): #todo updates all error messages
#     if arderr:
#         ArduinoError = True  
#     return

# Input functions
def readSerial(ser):
    '''takes in serial input string from input channel and updates relevant variables'''
    try: # this try except logic automatically reattempts to connect if connection lost
        ser.open() #open new port to input serial input channel
    except IOError: 
        ser.close()
        ser.open()
    except SerialException:
        serialReadError = True
        print('serial device not found/configurable')
    ln = ser.readline().split()
    if not ser.isOpen(): # This code should not be reache
        serialReadError = True # flag serial read error
        print('serial not read, something is wrong with code lol') 
    return ln

# output functions
ac_duty_cycle = lambda perc: -0.8 * perc + 85 

# main function, a big while loop
def main()
    while True:
        # fetch and update input values
        ign_switch = GPIO.input(2)
        ac_switch = GPIO.input(4)
        ac_over_pressure_switch = GPIO.input(14)
        dimmer_switch = GPIO.input(15)
        
        usb0_data = readSerial(usb0) #from usb0
        cell_volts = np.array(usb0_data[0:191]) # update cellvolts
        batt_temps = np.array(usb0_data[192:208]) # update batttemps
        air_temp = usb0_data[210] #check airtemp
        #motor_inverter_temp?
        #motor_stator_temp?
        
        # low_pressure
        
        # very often used variables
        cell_mean = np.mean(cell_volts)
        batt_temp_mean = np.mean(batt_temps)
        
        # UPDATING ERRORS
        acOverPressureError = (not ac_over_pressure_switch) # TODO ASK KEVIN WHAT THIS MEANS
        battVoltError = np.any((cell_volts < 3.2)|(cell_volts > 4.19)) # bad voltages
        battTempError = np.any(batt_temps > 45) # bad temps
        motorInverterTempError = motor_inverter_temp > 80
        motorStatorTempError = motor_stator_temp > 90
        battVoltDevError = np.any((cell_volts < cell_mean - 0.02) | 
                                  (cell_volts > cell_mean + 0.02)) #TODO: consider making mean a set variable
        battTempDevError = np.any((batt_temps < batt_temp_mean - 5) | 
                                  (batt_temps > batt_temp_mean + 5))
        ArduinoError = usb0_data[209] # gets arduino errors as boolean
        
        # outputs/switches
        if check_errors(): # if any error switch off TODO check if this is actually safe LOL
            GPIO.output(21, 0) # charger off
            GPIO.output(20, 0) # twelve volt charger off
            GPIO.output(26, 0) # pump off
            GPIO.output(16, 0) # ac comp off
            continue           # skips rest of while loop
        if ign_switch: # if ignition on
            GPIO.output(20, 1) # 12v charger on
        
        if GPIO.input(3): # checks charger switch, "isCharging"
            GPIO.output(21,1) # charger on
            GPIO.output(26,1) # pump on
            
        if 23 <= batt_temp_mean <= 30: # battery fan logic
            fan_pwm.ChangeDutyCycle((batt_temp_mean - 23) / 0.16)
        elif 30 < batt_temp_mean < 40:
            fan_pwm.ChangeDutyCycle(75)
            ac_comp_pwm.ChangeDutyCycle(ac_duty_cycle(50))
        elif 40 < batt_temp_mean:
            fan_pwm.ChangeDutyCycle(100)
            ac_comp_pwm.ChangeDutyCycle(ac_duty_cycle(100)) 
            print('batt temps above 40')
            
        if ac_switch:
            ac_comp_pwm.ChangeDutyCycle(ac_duty_cycle(100)) #TODO check if this is the same ac comp for batts
            
        sleep(0.1)
        
if __name__ == "__main__":
    main()


    
