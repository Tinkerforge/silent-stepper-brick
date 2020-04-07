using System;
using System.Threading;
using Tinkerforge;

class Example
{
	private static string HOST = "localhost";
	private static int PORT = 4223;
	private static string UID = "XXYYZZ"; // Change XXYYZZ to the UID of your Silent Stepper Brick

	static void Main()
	{
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickSilentStepper ss = new BrickSilentStepper(UID, ipcon); // Create device object

		ipcon.Connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		ss.SetMotorCurrent(800); // 800 mA
		ss.SetStepConfiguration(BrickSilentStepper.STEP_RESOLUTION_8,
		                        true); // 1/8 steps (interpolated)
		ss.SetMaxVelocity(2000); // Velocity 2000 steps/s

		// Slow acceleration (500 steps/s^2),
		// Fast deacceleration (5000 steps/s^2)
		ss.SetSpeedRamping(500, 5000);

		ss.Enable(); // Enable motor power
		ss.SetSteps(60000); // Drive 60000 steps forward

		Console.WriteLine("Press enter to exit");
		Console.ReadLine();

		// Stop motor before disabling motor power
		ss.Stop(); // Request motor stop
		ss.SetSpeedRamping(500, 5000); // Fast deacceleration (5000 steps/s^2) for stopping
		Thread.Sleep(400); // Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
		ss.Disable(); // Disable motor power

		ipcon.Disconnect();
	}
}
