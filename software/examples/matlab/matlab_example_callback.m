function matlab_example_callback()
    import com.tinkerforge.IPConnection;
    import com.tinkerforge.BrickSilentStepper;

    HOST = 'localhost';
    PORT = 4223;
    UID = 'XXYYZZ'; % Change XXYYZZ to the UID of your Silent Stepper Brick

    ipcon = IPConnection(); % Create IP connection
    ss = handle(BrickSilentStepper(UID, ipcon), 'CallbackProperties'); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Register position reached callback to function cb_position_reached
    set(ss, 'PositionReachedCallback', @(h, e) cb_position_reached(e));

    ss.setStepConfiguration(BrickSilentStepper.STEP_RESOLUTION_8, ...
                            true); % 1/8 steps (interpolated)
    ss.enable(); % Enable motor power
    ss.setSteps(1); % Drive one step forward to get things going

    input('Press key to exit\n', 's');

    % Stop motor before disabling motor power
    ss.stop(); % Request motor stop
    ss.setSpeedRamping(500, 5000); % Fast deacceleration (5000 steps/s^2) for stopping
    pause(0.4); % Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
    ss.disable(); % Disable motor power

    ipcon.disconnect();
end

% Use position reached callback to program random movement
function cb_position_reached(e)
    ss = e.getSource();

    if randi([0, 1])
        steps = randi([1000, 5000]); % steps (forward);
        fprintf('Driving forward: %g steps\n', steps);
    else
        steps = randi([-5000, -1000]); % steps (backward);
        fprintf('Driving backward: %g steps\n', steps);
    end

    vel = randi([200, 2000]); % steps/s
    acc = randi([100, 1000]); % steps/s^2
    dec = randi([100, 1000]); % steps/s^2
    fprintf('Configuration (vel, acc, dec): %g, %g, %g\n', vel, acc, dec);

    ss.setSpeedRamping(acc, dec);
    ss.setMaxVelocity(vel);
    ss.setSteps(steps);
end
