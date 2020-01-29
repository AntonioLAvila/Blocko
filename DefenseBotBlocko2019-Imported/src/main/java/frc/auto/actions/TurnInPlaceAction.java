package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Drive;
import frc.subsystems.Drive.DriveState;
import frc.util.DriveSignal;

public class TurnInPlaceAction implements Action {
    private Drive drive = Drive.getInstance();
    private double tolerance = 1.0;
    private double timeout = 2.0;
    private double startTime;

    //Degrees
    public TurnInPlaceAction(double angle, boolean relative, double tolerance, double timeout) {
        if(relative) {
            drive.setRelativeAngle(angle);
        } else {
            drive.setAbsoluteAngle(angle);
        }
        this.tolerance = tolerance;
        this.timeout = timeout;
    }

    public TurnInPlaceAction(double angle, boolean relative, double timeout) {
        this(angle, relative, 1.0, timeout);
    }

    public TurnInPlaceAction(double angle, boolean relative) {
        this(angle, relative, 1.0, 2.0);
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        drive.setDriveState(DriveState.TURN_IN_PLACE);
        System.out.println("TIP start");
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getHeading() - drive.getAbsAngleSetpoint()) < tolerance || Timer.getFPGATimestamp() - startTime >= timeout;
    }

    @Override
    public void done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }
}