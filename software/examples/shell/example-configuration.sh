#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XXYYZZ # Change XXYYZZ to the UID of your Silent Stepper Brick

tinkerforge call silent-stepper-brick $uid set-motor-current 800 # 800 mA
tinkerforge call silent-stepper-brick $uid set-step-configuration step-resolution-8 true # 1/8 steps (interpolated)
tinkerforge call silent-stepper-brick $uid set-max-velocity 2000 # Velocity 2000 steps/s

# Slow acceleration (500 steps/s^2),
# Fast deacceleration (5000 steps/s^2)
tinkerforge call silent-stepper-brick $uid set-speed-ramping 500 5000

tinkerforge call silent-stepper-brick $uid enable # Enable motor power
tinkerforge call silent-stepper-brick $uid set-steps 60000 # Drive 60000 steps forward

echo "Press key to exit"; read dummy

# Stop motor before disabling motor power
tinkerforge call silent-stepper-brick $uid stop # Request motor stop
tinkerforge call silent-stepper-brick $uid set-speed-ramping 500 5000 # Fast deacceleration (5000 steps/s^2) for stopping
sleep 0.4 # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
tinkerforge call silent-stepper-brick $uid disable # Disable motor power
