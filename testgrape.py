#!/usr/bin/env python

import logging
logging.basicConfig(level=logging.DEBUG)
from grape import *

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
