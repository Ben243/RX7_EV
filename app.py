import RPi.GPIO as GPIO
from gpiozero import LED, Button
import subprocess
import os, sys
import threading
import serial
import can
from time import sleep
import numpy as np

import json
import random
from datetime import datetime

from flask import Flask, Response, render_template

application = Flask(__name__) # initialize flask

'''global variables'''
speed = 999
rpm = 999
cell_volts = np.zeros(192) #[0 for x in range(192)] # 192, or 96*2 individual values
batt_temps = np.zeros(16) #[0 for x in range(16)] # 16 battery temps
# errmsg = False # arduino errors, see error declarations
air_temp = 0
motor_inverter_temp = 0
motor_stator_temp = 0
high_pressure_refrig = 0
lo_pressure_refrig = 0
temp_refrig = [] # count unsure TODO CHECK for final count

'''
SERIAL INITIALIZATION
'''
# inputs separated by spaces:
# cell volts, lv battery voltage, cell temps, errmsg airtemp
usb0 = serial.Serial() # usb serial input 1
usb0.baudrate = 19200
usb0.port = '/dev/ttyUSB0'
usb0.timeout = 1
# usb1 = serial.Serial('/dev/ttyUSB1', 9600, timeout=1) 
# ser = serial.Serial("/dev/ttyS0",115200,timeout=1)

def readSerial(ser):
    '''takes in serial input string from input channel and updates relevant variables'''
    try: # this try except logic automatically reattempts to connect if connection lost
        ser.open() #open new port to input serial insput channel
    except IOError: 
        ser.close()
        ser.open()
    except SerialException:
        # global serialReadError
        # serialReadError = True
        print('serial device not found/configurable')
        return
    if not ser.isOpen(): # TODO remove after debugging
        serialReadError = True # flag serial read error
        print('serial not read, something is wrong with code lol') 
        return
    ln = ser.readline().split()
    return ln

#flask stuff
@application.route('/')
def index():
    return render_template('index.html')

@application.route('/data')
def data():
    def generate_values():
        #global variable declaration
        # global ign_switch
        # global charger_switch
        # global ac_switch
        # global ac_over_pressure_switch
        # global dimmer_switch
        global speed
        global rpm
        global cell_volts
        global batt_temps
        # #global errmsg
        global air_temp
        # global motor_inverter_temp
        # global motor_stator_temp
        # global high_pressure_refrig
        # global lo_pressure_refrig
        # global temp_refrig
        # global ac_comp_pwm
        # global chiller_pwm
        # global cabin_pwm
        # global fan_pwm
        while True:
            usb0_data = readSerial(usb0) #from usb0
            # input format is speed rpm ignition charging ac_on ac_pressure dimmer cellvolt_avg batttemp_avg 
            # cell_volts = np.array(usb0_data[0:191]) # update cellvolts
            # batt_temps = np.array(usb0_data[192:208]) # update batttemps
            # air_temp = usb0_data[210] #check airtemp
            
            
            # cell_mean = np.mean(cell_volts)
            # batt_temp_mean = np.mean(batt_temps)
            # max_cellv_dev =  max(np.abs(cell_mean - cell_volts))
            # max_batt_temp_dev = max(np.abs(batt_temp_mean - batt_temps))
            
            # dummy values section
            speed = (speed + 1) % 100
            rpm = (rpm+0.25) % 8

            json_data = json.dumps(
                {'speed': speed,
                'rpm': rpm,
                # 'ignition': ign_switch, 
                # 'charging': charger_switch, 
                # 'twelvev': GPIO.input(20),
                # 'avgcellvolts': cell_mean, 
                # 'avgbatttemps': batt_temp_mean, 
                # 'cellvoltdevmax': max_cellv_dev,
                # 'batttempdevmax': max_batt_temp_dev,
                'airtemp': air_temp,
                # 'accomp': 99, 
                # 'pump': 99, 
                # 'allerrors': 1,
                })
            yield f"data:{json_data}\n\n"
            sleep(0.1) # update speed   
    return Response(generate_values(), mimetype='text/event-stream')