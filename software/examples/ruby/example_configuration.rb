#!/usr/bin/env ruby
# -*- ruby encoding: utf-8 -*-

require 'tinkerforge/ip_connection'
require 'tinkerforge/brick_silent_stepper'

include Tinkerforge

HOST = 'localhost'
PORT = 4223
UID = 'XXYYZZ' # Change XXYYZZ to the UID of your Silent Stepper Brick

ipcon = IPConnection.new # Create IP connection
ss = BrickSilentStepper.new UID, ipcon # Create device object

ipcon.connect HOST, PORT # Connect to brickd
# Don't use device before ipcon is connected

ss.set_motor_current 800 # 800 mA
ss.set_step_configuration BrickSilentStepper::STEP_RESOLUTION_8, \
                          true # 1/8 steps (interpolated)
ss.set_max_velocity 2000 # Velocity 2000 steps/s

# Slow acceleration (500 steps/s^2),
# Fast deacceleration (5000 steps/s^2)
ss.set_speed_ramping 500, 5000

ss.enable # Enable motor power
ss.set_steps 60000 # Drive 60000 steps forward

puts 'Press key to exit'
$stdin.gets

# Stop motor before disabling motor power
ss.stop # Request motor stop
ss.set_speed_ramping 500, 5000 # Fast deacceleration (5000 steps/s^2) for stopping
sleep 0.4 # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
ss.disable # Disable motor power

ipcon.disconnect
