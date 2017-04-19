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
        ss.setStepConfiguration(Tinkerforge.BrickSilentStepper.STEP_RESOLUTION_8,
                                true); // 1/8 steps (interpolated)
        ss.enable(); // Enable motor power
        ss.setSteps(1); // Drive one step forward to get things going
    }
);

// Register position reached callback
ss.on(Tinkerforge.BrickSilentStepper.CALLBACK_POSITION_REACHED,
    // Use position reached callback to program random movement
    function (position) {
        if(Math.floor(Math.random()*2)) {
            var steps = Math.floor((Math.random()*5000)+1000); // steps (forward);
            console.log('Driving forward: '+steps+' steps');
        }
        else {
            var steps = Math.floor((Math.random()*(-1000))+(-5000)); // steps (backward);
            console.log('Driving backward: '+steps+' steps');
        }

        var vel = Math.floor((Math.random()*2000)+200); // steps/s
        var acc = Math.floor((Math.random()*1000)+100); // steps/s^2
        var dec = Math.floor((Math.random()*1000)+100); // steps/s^2
        console.log('Configuration (vel, acc, dec): '+vel+', '+acc+', '+dec);

        ss.setSpeedRamping(acc, dec);
        ss.setMaxVelocity(vel);
        ss.setSteps(steps);
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
