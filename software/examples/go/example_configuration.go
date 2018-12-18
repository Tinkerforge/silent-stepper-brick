package main

import (
	"fmt"
	"github.com/tinkerforge/go-api-bindings/ipconnection"
	"github.com/tinkerforge/go-api-bindings/silent_stepper_brick"
)

const ADDR string = "localhost:4223"
const UID string = "XXYYZZ" // Change XXYYZZ to the UID of your Silent Stepper Brick.

func main() {
	ipcon := ipconnection.New()
	defer ipcon.Close()
	ss, _ := silent_stepper_brick.New(UID, &ipcon) // Create device object.

	ipcon.Connect(ADDR) // Connect to brickd.
	defer ipcon.Disconnect()
	// Don't use device before ipcon is connected.

	ss.SetMotorCurrent(800) // 800mA
	ss.SetStepConfiguration(silent_stepper_brick.StepResolution8,
		true) // 1/8 steps (interpolated)
	ss.SetMaxVelocity(2000) // Velocity 2000 steps/s

	// Slow acceleration (500 steps/s^2),
	// Fast deacceleration (5000 steps/s^2)
	ss.SetSpeedRamping(500, 5000)

	ss.Enable()        // Enable motor power
	ss.SetSteps(60000) // Drive 60000 steps forward

	fmt.Print("Press enter to exit.")
	fmt.Scanln()

	ss.Disable()
}
