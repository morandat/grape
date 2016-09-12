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

display.print_str("Hello grape ... it's so fun to play with you")

for i in range(5):
    display.led(Display.RED, i%2 == 1)
    display.led(Display.GREEN, i%2 == 0)
    sleep(.3)

display.clear_screen()
display.print_str("Hello grape ...")
display.home(1)
display.print_str("Bye")
display.print_at(0, 0, "X")
display.print_at(0, 15, "X")
display.print_at(1, 0, "X")
display.print_at(1, 15, "X")


#GPIO.setup(4, GPIO.IN)
#GPIO.setup(14, GPIO.IN)
while True:
    sleep(.01)
    display.interrupt(INTERUPT_PIN)
    #if GPIO.input(4):
        #print("OK")
    #if GPIO.input(14):
        #print("OK")
    #if GPIO.input(15):
        #print("OK")

display.print_str("Hello grape ...")
