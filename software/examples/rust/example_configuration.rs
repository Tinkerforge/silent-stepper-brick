use std::{error::Error, io};

use tinkerforge::{ipconnection::IpConnection, silent_stepper_brick::*};

const HOST: &str = "127.0.0.1";
const PORT: u16 = 4223;
const UID: &str = "XXYYZZ"; // Change XXYYZZ to the UID of your Silent Stepper Brick

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection
    let silent_stepper_brick = SilentStepperBrick::new(UID, &ipcon); // Create device object

    ipcon.connect(HOST, PORT).recv()??; // Connect to brickd
                                        // Don't use device before ipcon is connected

    silent_stepper_brick.set_motor_current(800); // 800mA
    silent_stepper_brick.set_step_configuration(SILENT_STEPPER_BRICK_STEP_RESOLUTION_8, true); // 1/8 steps (interpolated)
    silent_stepper_brick.set_max_velocity(2000); // Velocity 2000 steps/s

    // Slow acceleration (500 steps/s^2),
    // Fast deacceleration (5000 steps/s^2)
    silent_stepper_brick.set_speed_ramping(500, 5000);

    silent_stepper_brick.enable(); // Enable motor power
    silent_stepper_brick.set_steps(60000); // Drive 60000 steps forward

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    silent_stepper_brick.disable();
    ipcon.disconnect();
    Ok(())
}
