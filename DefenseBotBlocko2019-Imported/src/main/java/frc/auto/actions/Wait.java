package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class Wait implements Action {
	private double timeout;
	private double startTime;
	
	public Wait(double timeout) {
		this.timeout = timeout;
	}
	
	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime >= timeout;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}


}
