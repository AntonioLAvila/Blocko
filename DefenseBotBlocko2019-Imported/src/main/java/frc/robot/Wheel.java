package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Wheel extends Joystick {

	public Wheel(int port) {
		super(port);
	}
	
	public double getWheel() {
		return getRawAxis(0);
	}
	
	public boolean getButton(int button) {
		return getRawButton(button);
    }
    
}