package frc.util;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;


public class PID{
    private static double kP, kI, kD;
    private double error = 0.0;
    private double output = 0.;
    private double sumError = 0.;
    private double prevError = 0.;
    private double minOutput = 0.;
    private double maxOutput = 0.;

    /**
     * 
     * @param p Proportional Gain
     * @param i Integral Gain
     * @param d Derivative Gain
     * @param min Minimum Output for example when using percent output -1.0
     * @param max Maximum Output ^^ 1.0
     */
    public PID(double p, double i, double d, double min, double max){
        this.kP = p;
        this.kI = i;
        this.kD = i;
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
	 * Basic PID calculator made by "yours truely" -OJ Simpson
     * @param setPoint distance in whatever units your encoder is measuring if you haven't yet, convert the ticks.
	 * @param measurement Current sensor reading
	 * @param dt Time of loop
	 * @return calculated voltage to motors
	 */
    public double calculate(double setPoint, double measurement, double dt){
        error = setPoint - measurement;

        sumError += ((error + prevError) / 2) * dt;

        double derivError = (error - prevError) / dt;

        output = kP * error + kI * sumError + kD * derivError;

        //housekeeping
        prevError = error;
        if(output > maxOutput) {
			output = maxOutput;
		} else if(output < minOutput) {
			output = minOutput;
        }
        
        System.out.println(output);
        return output;
    }

    public void setMinMaxOutputs(double min, double max){
        if(minOutput > maxOutput) {
			System.out.println("\"Hey dummy, you made minOutput bigger than maxOutput\" -Maverick Zheng");
		}
		minOutput = min;
		maxOutput = max;
    }

    public void reset(){
        sumError = 0.;
        prevError = 0.;
        output = 0.;
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
		return output;
	}
}