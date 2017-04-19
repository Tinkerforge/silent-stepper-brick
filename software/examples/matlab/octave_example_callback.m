function octave_example_callback()
    more off;

    HOST = "localhost";
    PORT = 4223;
    UID = "XXYYZZ"; % Change XXYYZZ to the UID of your Silent Stepper Brick

    ipcon = java_new("com.tinkerforge.IPConnection"); % Create IP connection
    ss = java_new("com.tinkerforge.BrickSilentStepper", UID, ipcon); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Register position reached callback to function cb_position_reached
    ss.addPositionReachedCallback(@cb_position_reached);

    ss.setStepConfiguration(ss.STEP_RESOLUTION_8, true); % 1/8 steps (interpolated)
    ss.enable(); % Enable motor power
    ss.setSteps(1); % Drive one step forward to get things going

    input("Press key to exit\n", "s");
    ss.disable();
    ipcon.disconnect();
end

% Use position reached callback to program random movement
function cb_position_reached(e)
    ss = e.getSource();

    if randi([0, 1])
        steps = randi([1000, 5000]); % steps (forward);
        fprintf("Driving forward: %g steps\n", steps);
    else
        steps = randi([-5000, -1000]); % steps (backward);
        fprintf("Driving backward: %g steps\n", steps);
    end

    vel = randi([200, 2000]); % steps/s
    acc = randi([100, 1000]); % steps/s^2
    dec = randi([100, 1000]); % steps/s^2
    fprintf("Configuration (vel, acc, dec): %g, %g, %g\n", vel, acc, dec);

    ss.setSpeedRamping(acc, dec);
    ss.setMaxVelocity(vel);
    ss.setSteps(steps);
end
