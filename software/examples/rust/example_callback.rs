use rand::{thread_rng, Rng};
use std::{error::Error, io, thread};
use tinkerforge::{ipconnection::IpConnection, silent_stepper_brick::*};

const HOST: &str = "127.0.0.1";
const PORT: u16 = 4223;
const UID: &str = "XXYYZZ"; // Change XXYYZZ to the UID of your Silent Stepper Brick

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection
    let silent_stepper_brick = SilentStepperBrick::new(UID, &ipcon); // Create device object

    ipcon.connect(HOST, PORT).recv()??; // Connect to brickd
                                        // Don't use device before ipcon is connected

    //Create listener for position reached events.
    let position_reached_listener = silent_stepper_brick.get_position_reached_receiver();
    // Spawn thread to handle received events. This thread ends when the silent_stepper_brick
    // is dropped, so there is no need for manual cleanup.
    let silent_stepper_brick_copy = silent_stepper_brick.clone(); //Device objects don't implement Sync, so they can't be shared between threads (by reference). So clone the device and move the copy.
    thread::spawn(move || {
        let mut rng = thread_rng();
        for _event in position_reached_listener {
            let steps = if rng.gen() {
                let steps = rng.gen_range(1000, 5001); // steps (forward)
                println!("Driving forward: {} steps", steps);
                steps
            } else {
                let steps = rng.gen_range(-5000, -999); // steps (backward)
                println!("Driving backward: {} steps", steps);
                steps
            };

            let vel = rng.gen_range(200, 2001); // steps/s
            let acc = rng.gen_range(100, 1001); // steps/s^2
            let dec = rng.gen_range(100, 1001); // steps/s^2

            println!("Configuration (vel, acc, dec): ({}, {}, {})", vel, acc, dec);

            silent_stepper_brick_copy.set_speed_ramping(acc, dec);
            silent_stepper_brick_copy.set_max_velocity(vel);
            silent_stepper_brick_copy.set_steps(steps);
        }
    });

    silent_stepper_brick.set_step_configuration(SILENT_STEPPER_BRICK_STEP_RESOLUTION_8, true); // 1/8 steps (interpolated)
    silent_stepper_brick.enable(); // Enable motor power
    silent_stepper_brick.set_steps(1); // Drive one step forward to get things going

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    silent_stepper_brick.disable();
    ipcon.disconnect();
    Ok(())
}
