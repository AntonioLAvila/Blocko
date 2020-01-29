package trajectory_lib;


public class TrajectoryConfig {
	
	public static final int SAMPLES_FAST = 1000;
    public static final int SAMPLES_LOW = SAMPLES_FAST * 10;
    public static final int SAMPLES_HIGH = SAMPLES_LOW * 10;
    
    public int samples;
    public double dt, max_vel, max_acc, max_jerk;
    
    public TrajectoryConfig(int samples, double dt, double max_vel, double max_acc, double max_jerk) {
    	this.samples = samples;
    	this.dt = dt;
    	this.max_vel = max_vel;
    	this.max_acc = max_acc;
        this.max_jerk = max_jerk;
    
    }


}
