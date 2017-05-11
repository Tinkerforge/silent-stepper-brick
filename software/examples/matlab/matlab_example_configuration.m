function matlab_example_configuration()
    import com.tinkerforge.IPConnection;
    import com.tinkerforge.BrickSilentStepper;

    HOST = 'localhost';
    PORT = 4223;
    UID = 'XXYYZZ'; % Change XXYYZZ to the UID of your Silent Stepper Brick

    ipcon = IPConnection(); % Create IP connection
    ss = handle(BrickSilentStepper(UID, ipcon), 'CallbackProperties'); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    ss.setMotorCurrent(800); % 800mA
    ss.setStepConfiguration(BrickSilentStepper.STEP_RESOLUTION_8, ...
                            true); % 1/8 steps (interpolated)
    ss.setMaxVelocity(2000); % Velocity 2000 steps/s

    % Slow acceleration (500 steps/s^2),
    % Fast deacceleration (5000 steps/s^2)
    ss.setSpeedRamping(500, 5000);

    ss.enable(); % Enable motor power
    ss.setSteps(60000); % Drive 60000 steps forward

    input('Press key to exit\n', 's');
    ss.disable();
    ipcon.disconnect();
end
