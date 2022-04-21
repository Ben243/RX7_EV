#!/usr/bin/env python3

# import RPi.GPIO as GPIO
# from gpiozero import LED, Button
import subprocess
import os, sys
# import psutil
from threading import Timer
# import serial
# import can
from time import sleep
import numpy as np

import json
import random
from datetime import datetime

from flask import Flask, Response, render_template

app = Flask(__name__) # initialize flask

# '''gpio'''
# def kill(channel):
#     app.do_teardown_appcontext()
    
# def menu_press(channel):
#     global menu_button
#     menu_button = (menu_button + 1) % 5
    # print("press! "+ str(menu_button))

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.add_event_detect(21, GPIO.FALLING, callback=menu_press,bouncetime=200) # gpio for toggling 
# GPIO.add_event_detect(13, GPIO.FALLING, callback=kill,bouncetime=200) # gpio for killing flask

'''global variables'''
menu_button = 1
ign_switch = 1
charger_switch = 1
ac_switch = 1
dimmer_switch = 0
speed = 99
motorvolts = 99
bms_volts = 99
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
# usb0 = serial.Serial() # usb serial input 1
# usb0.baudrate = 19200
# usb0.port = '/dev/ttyUSB0'
# usb0.timeout = 1
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

'''helper functions'''
def pi_temp():
    # return (float) (os.popen("vcgencmd measure_temp").readline()[5:9])
    return 59.0
    
def batt_perc(value):
    if (value <= 3.4):
        return 1
    elif (value <= 3.6):
        return 5
    elif (value <= 3.85):
        return 15
    elif (value <= 3.9):
        return 30
    elif (value <= 3.95):
        return 40
    elif (value <= 4):
        return 50
    elif (value <= 4.05):
        return 65
    elif (value <= 4.1):
        return 85
    elif (value <= 4.15):
        return 91
    else: return 96


def open_browser(channel):
    # sleep(.01)
    os.system('DISPLAY=:0 chromium-browser --kiosk --start-fullscreen --disable-infobars http://localhost:5000')
    # subprocess.run("DISPLAY=:0 chromium-browser --kiosk http://localhost:5000".split())


def generate_values():
    # open_browser()
    '''main loop logic'''
    #global variable declaration
    global menu_button
    global ign_switch
    global charger_switch
    global ac_switch
    # global ac_over_pressure_switch
    # global dimmer_switch
    global speed
    global motorvolts
    global bms_volts
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

    counter = 0
    while True:
        # dummy values section, comment out before posting
        speed = int(np.round((speed + 1) % 180, 1))
        motorvolts =  (motorvolts + 1) % 500
        motorvoltsbuffer = int(np.round(150 * np.sin(.02*motorvolts) + 90))
        bms_volts = [3, 3.1, 3.5, 3.6, 3.8, 3.9, 3.97, 4.00, 4.10, 4.2]
        # print(np.round(150 * np.sin(.02*(motorvolts)) + 90))
        cell_volts = np.random.normal(loc=3.6, size=191) #dummy
        batt_temps = np.random.normal(loc=30, size=16) #dummy

        # true input section
        # usb0_data = readSerial(usb0) #from usb0
        # input format is speed ignition charging ac_on ac_pressure dimmer cellvolt_avg batttemp_avg 
        # cell_volts = np.array(usb0_data[0:191]) # update cellvolts
        # batt_temps = np.array(usb0_data[192:208]) # update batttemps
        # air_temp = usb0_data[210] #check airtemp
        
        # calculations

        cell_mean = np.round(np.mean(cell_volts), 2)
        batt_temp_mean = np.round(np.mean(batt_temps), 2)
        max_cellv_dev =  np.round(max(np.abs(cell_mean - cell_volts)), 2)
        max_batt_temp_dev = np.round(max(np.abs(batt_temp_mean - batt_temps)), 2)

        
        # if counter == 0:
        json_data = json.dumps(
            {"menu": menu_button,
            'speed': speed,
            'motorvolts': motorvoltsbuffer,
            'ignition': ign_switch, 
            'charging': charger_switch, 
            # 'twelvev': 1, #gpio.input(12)
            'bms_volts': bms_volts[counter],
            'vperc': batt_perc(bms_volts[counter]),
            'avgcellvolts': cell_mean, 
            'avgbatttemps': batt_temp_mean, 
            'cellvoltdevmax': max_cellv_dev,
            'batttempdevmax': max_batt_temp_dev,
            'airtemp': air_temp,
            'accomp': 99, 
            'pump': 99, 
            'pitemp': int(pi_temp()),
            # 'piload': psutil.cpu_percent(),
            'piload': .99,
            # 'allerrors': 0,
            })
        yield f"data:{json_data}\n\n"
        # print(speed)
        counter = (counter + 1) % 10
        sleep(0.15) # multiply by 10 to send message


#flask stuff
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():  
    return Response(generate_values(), mimetype='text/event-stream')
    
if __name__ == '__main__':
    # Timer(0.1, open_browser).starst()
    app.run(debug=False, threaded=True)