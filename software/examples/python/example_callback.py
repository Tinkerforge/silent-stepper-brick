#!/usr/bin/env python
# -*- coding: utf-8 -*-

HOST = "localhost"
PORT = 4223
UID = "XXYYZZ" # Change XXYYZZ to the UID of your Silent Stepper Brick

import random

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_silent_stepper import BrickSilentStepper

# Use position reached callback to program random movement
def cb_position_reached(position):
    if random.randint(0, 1):
        steps = random.randint(1000, 5000) # steps (forward)
        print("Driving forward: " + str(steps) + " steps")
    else:
        steps = random.randint(-5000, -1000) # steps (backward)
        print("Driving backward: " + str(steps) + " steps")

    vel = random.randint(200, 2000) # steps/s
    acc = random.randint(100, 1000) # steps/s^2
    dec = random.randint(100, 1000) # steps/s^2
    print("Configuration (vel, acc, dec): " + str((vel, acc, dec)))

    ss.set_speed_ramping(acc, dec)
    ss.set_max_velocity(vel)
    ss.set_steps(steps)

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    ss = BrickSilentStepper(UID, ipcon) # Create device object

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Register position reached callback to function cb_position_reached
    ss.register_callback(ss.CALLBACK_POSITION_REACHED, cb_position_reached)

    ss.set_step_configuration(ss.STEP_RESOLUTION_8, True) # 1/8 steps (interpolated)
    ss.enable() # Enable motor power
    ss.set_steps(1) # Drive one step forward to get things going

    raw_input("Press key to exit\n") # Use input() in Python 3
    ss.disable()
    ipcon.disconnect()
