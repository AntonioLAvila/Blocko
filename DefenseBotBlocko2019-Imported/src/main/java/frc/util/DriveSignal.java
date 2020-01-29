package frc.util;

/**
 * 
 * @author Maverick Zhang
 * A class based on 254's DriveSignal class which holds the voltage for each
 * side of drive and whether or not we want to brake.
 *
 */
public class DriveSignal {
	
	protected double leftSignal;
	protected double rightSignal;
	protected boolean brakeMode;
	
	public DriveSignal(double leftSignal, double rightSignal, boolean brakeMode) {
		this.leftSignal = leftSignal;
		this.rightSignal = rightSignal;
		this.brakeMode = brakeMode;
	}
	
	public DriveSignal(double leftSignal, double rightSignal) {
		this(leftSignal, rightSignal, false);
	}
	
	public static DriveSignal NEUTRAL = new DriveSignal(0.0, 0.0);
	public static DriveSignal BRAKE = new DriveSignal(0.0, 0.0, true);
	
	public double getLeft() {
		return leftSignal;
	}
	
	public double getRight() {
		return rightSignal;
	}
	
	public boolean getBrakeMode() {
		return brakeMode;
	}

}
