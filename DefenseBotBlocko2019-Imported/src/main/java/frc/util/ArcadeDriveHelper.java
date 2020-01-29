package frc.util;

import frc.util.DriveSignal;

/**
 * 
 * @author WPILIB
 * This is just a helper function for arcadeDrive functionality. The code is taken from WPILIB.
 *
 */
public class ArcadeDriveHelper {
	public static final double kDefaultDeadband = 0.02;
	public static double m_deadband = kDefaultDeadband;
	public static final double kDefaultMaxOutput = 1.0;
	public static double m_maxOutput = kDefaultMaxOutput;
	
	public ArcadeDriveHelper() {
	}
	
	protected double limit(double value) {
		    if (value > 1.0) {
		      return 1.0;
		    }
		    if (value < -1.0) {
		      return -1.0;
		    }
		    return value;
	}
	
	protected double applyDeadband(double value, double deadband) {
		    if (Math.abs(value) > deadband) {
		      if (value > 0.0) {
		        return (value - deadband) / (1.0 - deadband);
		      } else {
		        return (value + deadband) / (1.0 - deadband);
		      }
		    } else {
		      return 0.0;
		    }
    }
	
	public DriveSignal arcadeDrive(double xSpeed, double zRotation) {
		return arcadeDrive(xSpeed, zRotation, true);
	}
	
	public DriveSignal arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
		xSpeed = limit(xSpeed);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = limit(zRotation);
    zRotation = applyDeadband(zRotation, m_deadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squaredInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

   
		    
		    DriveSignal signal;

		    signal= new DriveSignal(-limit(leftMotorOutput) * m_maxOutput , -limit(rightMotorOutput) * m_maxOutput);
		    
		    return signal;
	}
}
