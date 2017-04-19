#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XXYYZZ # Change XXYYZZ to the UID of your Silent Stepper Brick

tinkerforge call silent-stepper-brick $uid set-motor-current 800 # 800mA
tinkerforge call silent-stepper-brick $uid set-step-configuration step-resolution-8 true # 1/8 steps (interpolated)
tinkerforge call silent-stepper-brick $uid set-max-velocity 2000 # Velocity 2000 steps/s

# Slow acceleration (500 steps/s^2),
# Fast deacceleration (5000 steps/s^2)
tinkerforge call silent-stepper-brick $uid set-speed-ramping 500 5000

tinkerforge call silent-stepper-brick $uid enable # Enable motor power
tinkerforge call silent-stepper-brick $uid set-steps 60000 # Drive 60000 steps forward

echo "Press key to exit"; read dummy

tinkerforge call silent-stepper-brick $uid disable
