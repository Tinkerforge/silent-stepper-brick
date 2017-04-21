import java.util.Random;

import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickSilentStepper;
import com.tinkerforge.TinkerforgeException;

public class ExampleCallback {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XXYYZZ to the UID of your Silent Stepper Brick
	private static final String UID = "XXYYZZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		// Note: Declare stepper final, so the listener can access it
		final BrickSilentStepper ss = new BrickSilentStepper(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Use position reached callback to program random movement
		ss.addPositionReachedListener(new BrickSilentStepper.PositionReachedListener() {
			Random random = new Random();

			public void positionReached(int position) {
				int steps = 0;

				if(random.nextInt(2) == 1) {
					steps = random.nextInt(4001) + 1000; // steps (forward)
				} else {
					steps = random.nextInt(5001) - 6000; // steps (backward)
				}

				int vel = random.nextInt(1801) + 200; // steps/s
				int acc = random.nextInt(901) + 100; // steps/s^2
				int dec = random.nextInt(901) + 100; // steps/s^2

				System.out.println("Configuration (vel, acc, dec): (" +
				                   vel + ", " + acc + ",  " + dec + ")");

				try {
					ss.setSpeedRamping(acc, dec);
					ss.setMaxVelocity(vel);
					ss.setSteps(steps);
				} catch(TinkerforgeException e) {
				}
			}
		});

		ss.setStepConfiguration(BrickSilentStepper.STEP_RESOLUTION_8,
		                        true); // 1/8 steps (interpolated)
		ss.enable(); // Enable motor power
		ss.setSteps(1); // Drive one step forward to get things going

		System.out.println("Press key to exit"); System.in.read();
		ss.disable();
		ipcon.disconnect();
	}
}
