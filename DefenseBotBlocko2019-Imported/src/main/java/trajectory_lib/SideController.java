package trajectory_lib;


/**
 * 
 * @author Maverick Zhang
 * Based on 254's synchronous PID controller. This controller essentially is PID, and has support
 * for continuous inputs, prevention of integral windup, and motion profiles. (kV is a feedforward constant 
 * that only works for motion profiling)
 */
public class SideController {
	private double kP, kI, kD, kV, kA;
	private double maxOutput = 1.0;
	private double minOutput = -1.0;
	private double maxInput = 0.0;
	private double minInput = 0.0;
	private boolean continuousInput = false; //For sensors that wrap around e.g. absolute encoder since values go from 0 to 359
	private double totalError = 0.0;
	private double error = 0.0;
	private double result = 0.0;
	private double lastInput = Double.NaN;
	private double deadband = 0.0;
	private Trajectory traj;
	private int index;
	
	public SideController(double kP, double kI, double kD, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
		kA = 0.0;
	}
	
	public SideController(double kP, double kI, double kD, double kV, double kA) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
		this.kA = kA;
	}
	
	public SideController() {
		
	}
	
	public void setAutoTrajectory(Trajectory traj) {
		this.traj = traj;
	}
	
	public SideController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = 0.0;
		kA = 0.0;
	}
	
	public double calculate(double pos_meas, double vel_meas, double dt) {
		if(dt < 1.0E-6) {
			dt = 1.0E-6;
		}
		lastInput = pos_meas;
		Segment current = traj.get(index);
		if(index < traj.length() - 1) {
			index++;
		}
		error = current.pos - pos_meas;
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
		
		if(index < traj.length() - 1) {
			result = kP * propError + kI * totalError + kD * (current.vel - vel_meas) + kV * current.vel + kA * current.acc;
		} else {
			result = kP * propError + kI * totalError + kD * (- vel_meas);
		}
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
	
	public void setPID(double kP, double kI, double kD, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
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
	
	public double getV() {
		return kV;
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
	
	public double getSetpoint() {
		return traj.get(index).pos;
	}
	
	public double getError() {
		return error;
	}
	
	public boolean onTarget(double tolerance) {
		return lastInput != Double.NaN && Math.abs(lastInput - getSetpoint()) < tolerance;
	}
	
	public void reset() {
		lastInput = Double.NaN;
		totalError = 0.0;
		result = 0.0;
		index = 0;
	}
	
	public void resetIntegrator() {
		totalError = 0.0;
	}
	
	public boolean isFinished(double tolerance) {
		return index == traj.length() - 1 && onTarget(tolerance);
	}
	
	public boolean isFinished() {
		return index == traj.length() - 1;
	}
	
	public Trajectory getTrajectory() {
		return traj;
	}
	
	public int currIndex() {
		return index;
	}
	
	

}
