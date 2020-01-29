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
        runAction(new FollowTrajectoryAction(Constants.path, false));
    }
    //from waypoint to waypoint thing but not pass through the points given
    
}