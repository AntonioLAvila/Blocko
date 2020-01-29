package frc.robot;

import java.util.List;

import trajectory_lib.AutoTrajectory;
import trajectory_lib.Waypoint;

public class AutoDriveData {
    public double yawAbs, txAbs, sign, x, y, a1, a2, angle, startHeading;
    public List<Waypoint> path1, path2;
    public AutoTrajectory traj1, traj2;
}