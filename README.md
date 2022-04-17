# RX7 EV

A project for the 21st century, the RX7 EV!

This repository holds the logic for controlling a raspberry pi as a live high refresh rate dashboard to manage this special car's onboard systems via serial ports and gpio pins.

The car's onboard battery management and temperature systems were managed by arduinos. The raspberry pi took the serial output data as sent by these arduinos, and through a straightforward implementation of SSE in python's flask module was able to display this information on a 21:9 aspect ratio (1920 x 720) display via chromium.
