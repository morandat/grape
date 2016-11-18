#!/usr/bin/env python

import logging
logging.basicConfig(level=logging.INFO)
from grape import *
from RPi import GPIO

for i, s in stacks.items():
    print("STACK: {0:o} {1}".format(i, s.devices))


STACK = 010
if False and stacks.has_key(STACK):
    stacks[STACK][PowerSwitch].stop_all()
    print(stacks[STACK][PowerSwitch].status())
    stacks[STACK][PowerSwitch][3] = True
    print(stacks[STACK][PowerSwitch].status())
    print(stacks[STACK][PowerMeter].current())
    print(stacks[STACK][PowerMeter].voltage())
    print(stacks[STACK][PowerMeter].power())


stacks[010][PowerSwitch].stop_all()
sleep(1)
for i, s in stacks.items():
    print "--- Stack ", i, "---" 
    print "Temperature: ", s[Temperature].get(), " celsius"
    print "Current: ", s[PowerMeter].current()
    print "Voltage: ", s[PowerMeter].voltage(), "mV"
    print "Power:", s[PowerMeter].power()
    print "Shunt:", s[PowerMeter].shunt()

sleep(1)
stacks[010][PowerSwitch].start_all()
sleep(1)

for i, s in stacks.items():
    print "--- Stack ", i, "---" 
    print "Temperature: ", s[Temperature].get(), " celsius"
    print "Current: ", s[PowerMeter].current()
    print "Voltage: ", s[PowerMeter].voltage(), "mV"
    print "Power:", s[PowerMeter].power()
    print "Shunt:", s[PowerMeter].shunt()

if False and display is not None:
    display.btn_handler = ButtonPrinter()

    try:
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
    except: pass


    #GPIO.setup(4, GPIO.IN)
    #GPIO.setup(14, GPIO.IN)
    while True:
        sleep(1)
        if False: display.interrupt(DSP_INTERUPT_PIN)
        print("waiting ...")
        #if GPIO.input(4):
            #print("OK")
        #if GPIO.input(14):
            #print("OK")
        #if GPIO.input(15):
            #print("OK")

    display.print_str("Hello grape ...")
