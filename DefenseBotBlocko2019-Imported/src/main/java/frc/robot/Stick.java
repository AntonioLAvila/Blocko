package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Stick extends Joystick {

	public Stick(int port) {
		super(port);
	}
	
	public double getThrottle() {
		return getRawAxis(1);
	}
	
	public double getTwist() {
		return getRawAxis(2);
	}
	
	public boolean getButton(int button) {
		return getRawButton(button);
	}
}