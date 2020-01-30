package frc.auto.modes;

import java.util.Arrays;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.FollowTrajectoryAction;
import frc.robot.Constants;
import trajectory_lib.Waypoint;

public class JustFollowATrajectory extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new FollowTrajectoryAction(Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(10.0, 10.0, 0.), new Waypoint(20., 10., 0.)), false));
    }
    //from waypoint to waypoint thing but not pass through the points given
    
}