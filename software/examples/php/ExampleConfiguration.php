<?php

require_once('Tinkerforge/IPConnection.php');
require_once('Tinkerforge/BrickSilentStepper.php');

use Tinkerforge\IPConnection;
use Tinkerforge\BrickSilentStepper;

const HOST = 'localhost';
const PORT = 4223;
const UID = 'XXYYZZ'; // Change XXYYZZ to the UID of your Silent Stepper Brick

$ipcon = new IPConnection(); // Create IP connection
$ss = new BrickSilentStepper(UID, $ipcon); // Create device object

$ipcon->connect(HOST, PORT); // Connect to brickd
// Don't use device before ipcon is connected

$ss->setMotorCurrent(800); // 800mA
$ss->setStepConfiguration(BrickSilentStepper::STEP_RESOLUTION_8,
                          TRUE); // 1/8 steps (interpolated)
$ss->setMaxVelocity(2000); // Velocity 2000 steps/s

// Slow acceleration (500 steps/s^2),
// Fast deacceleration (5000 steps/s^2)
$ss->setSpeedRamping(500, 5000);

$ss->enable(); // Enable motor power
$ss->setSteps(60000); // Drive 60000 steps forward

echo "Press key to exit\n";
fgetc(fopen('php://stdin', 'r'));
$ss->disable();
$ipcon->disconnect();

?>
