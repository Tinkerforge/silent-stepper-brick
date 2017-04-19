#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XXYYZZ # Change XXYYZZ to the UID of your Silent Stepper Brick

# Use position reached callback to program random movement
tinkerforge dispatch silent-stepper-brick $uid position-reached\
 --execute "echo Changing configuration;
            tinkerforge call silent-stepper-brick $uid set-max-velocity $(((RANDOM%1800)+1200));
            tinkerforge call silent-stepper-brick $uid set-speed-ramping $(((RANDOM%900)+100)) $(((RANDOM%900)+100));
            if [ $((RANDOM % 2)) -eq 1 ];
            then tinkerforge call silent-stepper-brick $uid set-steps $(((RANDOM%4000)+1000));
            else tinkerforge call silent-stepper-brick $uid set-steps $(((RANDOM%4000)-5000));
            fi" &

tinkerforge call silent-stepper-brick $uid set-step-configuration step-resolution-8 true # 1/8 steps (interpolated)
tinkerforge call silent-stepper-brick $uid enable # Enable motor power
tinkerforge call silent-stepper-brick $uid set-steps 1 # Drive one step forward to get things going

echo "Press key to exit"; read dummy

tinkerforge call silent-stepper-brick $uid disable

kill -- -$$ # Stop callback dispatch in background
