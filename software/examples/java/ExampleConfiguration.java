import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickSilentStepper;

public class ExampleConfiguration {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XXYYZZ to the UID of your Silent Stepper Brick
	private static final String UID = "XXYYZZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickSilentStepper ss = new BrickSilentStepper(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		ss.setMotorCurrent(800); // 800mA
		ss.setStepConfiguration(BrickSilentStepper.STEP_RESOLUTION_8,
		                        true); // 1/8 steps (interpolated)
		ss.setMaxVelocity(2000); // Velocity 2000 steps/s

		// Slow acceleration (500 steps/s^2),
		// Fast deacceleration (5000 steps/s^2)
		ss.setSpeedRamping(500, 5000);

		ss.enable(); // Enable motor power
		ss.setSteps(60000); // Drive 60000 steps forward

		System.out.println("Press key to exit"); System.in.read();
		ss.disable();
		ipcon.disconnect();
	}
}
