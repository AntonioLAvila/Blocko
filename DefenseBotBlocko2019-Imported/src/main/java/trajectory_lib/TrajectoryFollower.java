package trajectory_lib;

import frc.util.DriveSignal;

public class TrajectoryFollower {
	private SideController left,right;
	private HeadingController pidHeading;
	private AutoTrajectory traj;
	
	private boolean reverse = false;
	private double left_offset, right_offset;
	
	public TrajectoryFollower(double sidekP, double sidekI, double sidekD, double sidekV, double sidekA, double kW) {
		this(sidekP, sidekI, sidekD, sidekV, sidekA, sidekP, sidekI, sidekD, sidekV, sidekA, kW, 0.0, 0.0);
	}
	
	public TrajectoryFollower(double leftkP, double leftkI, double leftkD, double leftkV, double leftkA,
			double rightkP, double rightkI, double rightkD, double rightkV, double rightkA,
			double kWP, double kWI, double kWD) {
		left = new SideController(leftkP, leftkI, leftkD, leftkV, leftkA);
		right = new SideController(rightkP, rightkI, rightkD, rightkV, rightkA);
		pidHeading = new HeadingController(kWP, kWI, kWD);
	}
	
	public void setTrajectory(AutoTrajectory traj) {
		this.traj = traj;
	}
	
	public void start(double left_meas, double right_meas, boolean reverse) {
		this.reverse = reverse;
		left_offset = left_meas;
		right_offset = right_meas;
		if(reverse) {
			left.setAutoTrajectory(traj.right);
			right.setAutoTrajectory(traj.left);
		} else {
			left.setAutoTrajectory(traj.left);
			right.setAutoTrajectory(traj.right);
		}
		reset();
	}

	public HeadingController getHeadingController() {
		return pidHeading;
	}
	
	//Make sure heading is degrees
	public DriveSignal calculate(double left_pos, double left_vel, double right_pos, double right_vel, double heading, double dt) {
		double l = left_pos - left_offset;
		double r = right_pos - right_offset;
		double leftSignal, rightSignal;
		
		if(reverse) {
			leftSignal = -left.calculate(-l, -left_vel, dt);
			rightSignal = - right.calculate(-r, -right_vel, dt);
			
			double angle = Utils.boundHalfDegrees(Math.toDegrees(left.getTrajectory().get(left.currIndex()).theta)- 180.0);
			double curr_heading = Utils.boundHalfDegrees(heading);
			
			double angleToTurn = pidHeading.calculate(angle, curr_heading, dt);
			
			leftSignal -= angleToTurn;
			rightSignal += angleToTurn;
			
			return new DriveSignal(leftSignal, rightSignal, true);
		}
		
		leftSignal = left.calculate(l, left_vel, dt);
		rightSignal = right.calculate(r, right_vel, dt);
		
		double angle = Utils.boundHalfDegrees(Math.toDegrees(left.getTrajectory().get(left.currIndex()).theta));
		double curr_heading = Utils.boundHalfDegrees(heading);
		
		double angleToTurn = pidHeading.calculate(angle, curr_heading, dt);
		
		leftSignal -= angleToTurn;
		rightSignal += angleToTurn;
		
		return new DriveSignal(leftSignal, rightSignal);
	}
	
	public void reset() {
		left.reset();
		right.reset();
		pidHeading.reset();
	}
	
	//Maybe change to ||?
	public boolean isFinished() {
		System.out.println(left.isFinished());
		return left.isFinished() && right.isFinished();
	}

}
