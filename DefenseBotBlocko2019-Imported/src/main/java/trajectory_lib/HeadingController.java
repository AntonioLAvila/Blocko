package trajectory_lib;



/**
 * 
 * @author Maverick Zhang
 * Based on 254's synchronous PID controller. This controller essentially is PID, and has support
 * for continuous inputs, prevention of integral windup, and motion profiles. (kV is a feedforward constant 
 * that only works for motion profiling) 
 */
public class HeadingController {
	private double kP, kI, kD;
	private double maxOutput = 1.0;
	private double minOutput = -1.0;
	private double maxInput = 180.0;
	private double minInput = -180.0;
	private boolean continuousInput = true; //For sensors that wrap around e.g. absolute encoder since values go from 0 to 359
	private double prevError = 0.0;
	private double totalError = 0.0;
	private double error = 0.0;
	private double result = 0.0;
	private double lastInput = Double.NaN;
	private double deadband = 0.0;
	
	public HeadingController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
	
	public HeadingController() {
		
	}
	
	/**
	 * One of the two calculate methods. This one is designed to only be
	 * used with a nonchanging setpoint. Does not have a feedforward
	 * @param measurement Current sensor reading
	 * @param dt Time of loop
	 * @return
	 */
	public double calculate(double setpoint, double measurement, double dt) {
		if(dt < 1.0E-6) {
			dt = 1.0E-6;
		}
		lastInput = measurement;
		error = setpoint - measurement;
		if(continuousInput) {
			if(Math.abs(error) > (maxInput - minInput) / 2.0) {
				if(error > 0.0) {
					error  = error - maxInput + minInput;
				} else {
					error  = error + maxInput - minInput;
				}
			}
		}
		
		if((error * kP < maxOutput) && (error * kP > minOutput)) {
			totalError += error * dt;
		} else {
			totalError = 0.0;
		}
		
		double propError;
		if(Math.abs(error) < deadband) {
			propError = 0.0;
		} else {
			propError = error;
		}
		
		result = kP * propError + kI * totalError + kD * (error - prevError) / dt;
		
		prevError = error;
		
		if(result > maxOutput) {
			result = maxOutput;
		} else if(result < minOutput) {
			result = minOutput;
		}
		
		return result;
	}
	
	public void setPID(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public double getP() {
		return kP;
	}
	
	public double getI() {
		return kI;
	}
	
	public double getD() {
		return kD;
	}
	
	public double get() {
		return result;
	}
	
	//When in continuous, min and max input are assumed to be the same point, so an if statement reduces the error as much
	//as possible
	public void setContinuous(boolean continuous) {
		continuousInput = continuous;
	}
	
	public void setDeadband(double deadband) {
		this.deadband = deadband;
	}
	
	public void setInputRange(double minInput, double maxInput) {
		if(minInput > maxInput) {
			System.out.println("Hey dummy, you made minInput bigger than maxInput");
		}
		this.minInput = minInput;
		this.maxInput = maxInput;
	}
	
	public void setOutputRange(double minOutput, double maxOutput) {
		if(minOutput > maxOutput) {
			System.out.println("Hey dummy, you made minOutput bigger than maxOutput");
		}
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
	}
	
	public double getError() {
		return error;
	}
	
	public void reset() {
		lastInput = Double.NaN;
		prevError = 0.0;
		totalError = 0.0;
		result = 0.0;
	}
	
	public void resetIntegrator() {
		totalError = 0.0;
	}
	

	
	
	
	

}

