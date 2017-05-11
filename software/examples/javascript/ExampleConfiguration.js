var Tinkerforge = require('tinkerforge');

var HOST = 'localhost';
var PORT = 4223;
var UID = 'XXYYZZ'; // Change XXYYZZ to the UID of your Silent Stepper Brick

var ipcon = new Tinkerforge.IPConnection(); // Create IP connection
var ss = new Tinkerforge.BrickSilentStepper(UID, ipcon); // Create device object

ipcon.connect(HOST, PORT,
    function (error) {
        console.log('Error: ' + error);
    }
); // Connect to brickd
// Don't use device before ipcon is connected

ipcon.on(Tinkerforge.IPConnection.CALLBACK_CONNECTED,
    function (connectReason) {
        ss.setMotorCurrent(800); // 800mA
        ss.setStepConfiguration(Tinkerforge.BrickSilentStepper.STEP_RESOLUTION_8,
                                true); // 1/8 steps (interpolated)
        ss.setMaxVelocity(2000); // Velocity 2000 steps/s

        // Slow acceleration (500 steps/s^2),
        // Fast deacceleration (5000 steps/s^2)
        ss.setSpeedRamping(500, 5000);

        ss.enable(); // Enable motor power
        ss.setSteps(60000); // Drive 60000 steps forward
    }
);

console.log('Press key to exit');
process.stdin.on('data',
    function (data) {
        ss.disable();
        ipcon.disconnect();
        process.exit(0);
    }
);
