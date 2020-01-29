package frc.auto.actions;

import java.util.List;

import frc.robot.Constants;
import frc.subsystems.Drive;
import frc.util.DriveSignal;
import trajectory_lib.AutoTrajectory;
import trajectory_lib.Trajectory;
import trajectory_lib.TrajectoryGenerator;
import trajectory_lib.Waypoint;

public class FollowTrajectoryAction implements Action {
    private Drive drive = Drive.getInstance();

    public FollowTrajectoryAction(AutoTrajectory traj, boolean reversed) {
        drive.setTrajectory(traj, reversed);
    }

    public FollowTrajectoryAction(List<Waypoint> path, boolean reversed) {
        Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), path);
        AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
        drive.setTrajectory(traj, reversed);
    }

    @Override
    public void start() {
        drive.startPathFollowing();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        
        return drive.pathIsFinished();
    }

    @Override
    public void done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }

}