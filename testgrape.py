#!/usr/bin/env python

import logging
logging.basicConfig(level=logging.INFO)
from grape import *
from RPi import GPIO

for i, s in stacks.items():
    print("STACK: {0:o} {1}".format(i, s.devices))

if False:
    stacks[0][PowerSwitch].stop_all()
    print(stacks[0][PowerSwitch].status())
    stacks[0][PowerSwitch][0] = True
    print(stacks[0][PowerSwitch].status())
    print(stacks[0][Temperature].get())

    print(stacks[0][PowerMeter].current())
    print(stacks[0][PowerMeter].voltage())
    print(stacks[0][PowerMeter].power())

display.btn_handler = ButtonPrinter()

for i in range(5):
    display.led(Display.RED, i%2 == 1)
    display.led(Display.GREEN, i%2 == 0)
    sleep(.3)

display.print_str("Hello grape ...")

while True:
    sleep(.3)
    print(display._read_state())
    #if GPIO.input(15):
    #    print("OK")

display.print_str("Hello grape ...")
