package frc.robot;

import java.util.Arrays;
import java.util.List;

import trajectory_lib.Waypoint;

public class Constants {
//---------------------DONT KNOW ANY OF THESE-------------------------

    public static final int LONG_CAN_TIMEOUT_MS = 100; //im miliseconds
    public static final int WHEEL_DIAMETER = 6;

    public static final int SHIFTER_SOLENOID_1 = 1;
    public static final int SHIFTER_SOLENOID_2 = 2;

    public static final int LEFT_FRONT_DRIVE_TALON = 9;
    public static final int LEFT_MID_DRIVE_VICTOR = 0;
    public static final int LEFT_REAR_DRIVE_VICTOR = 12;

    public static final int RIGHT_FRONT_DRIVE_TALON = 6;
    public static final int RIGHT_MID_DRIVE_VICTOR = 7;
    public static final int RIGHT_REAR_DRIVE_VICTOR = 10;

    public static final int INTAKE_SOL_1 = 3;
    public static final int INTAKE_SOL_2 = 4;

    public static final int EJECT_SOL = 0;

    public static final int DRIVE_IMU = 1;

    public static final int DRIVE_LEFT_ENCODER_1 = 13;
    public static final int DRIVE_LEFT_ENCODER_2 = 14;
    public static final int DRIVE_RIGHT_ENCODER_1 = 15;
    public static final int DRIVE_RIGHT_ENCODER_2 = 16;

   // public static final int DRIVE_IMU = 9;

    
    public static final double DRIVE_ALLOWED_SPEED = 13.2344; //in ft/sec
    public static final double DRIVE_MAX_ACC = 10.; //in ft/sec^2 ?
    public static final double DRIVE_MAX_JERK = 60.;
    public static final double TEST_ALLOWED_SPEED = 13.2344; //in ft/sec
    public static final double TRAJECTORY_FOLLOWER_KP = 0.17;//.15 //proportional gain
    public static final double TRAJECTORY_FOLLOWER_KI = 0.; //Integral gain
    public static final double TRAJECTORY_FOLLOWER_KD = 0.0;//0.0 //derivative gain
    public static final double TRAJECTORY_FOLLOWER_KV = 0.0;
    public static final double TRAJECTORY_FOLLOWER_KA = 0.000; //acceleration gain
    public static final double TRAJECTORY_FOLLOWER_KW = 0.004;

    public static final double HEADING_CONTROLLER_KP = 0.004;
    public static final double HEADING_CONTROLLER_KI = 0.0000;
    public static final double HEADING_CONTROLLER_KD = 0.00;
    public static final double WHEEL_BASE = 2.6; //ft front to back

    public static final double LOOPER_DT = 0.005; //loop time

    public static final double ENCODER_CONVERSION = 1 / (360.0 * 1440.0); //encoder to tick/1deg
    //public static final double ENCODER_UNIT_TO_DEG = 1 / ENCODER_CONVERSION;

    //test
    public static final int LEFT_RAILGUN = 5;
    public static final int RIGHT_RAILGUN = 8;

    public static final int RAILGUN_ENC1 = 0;
    public static final int RAILGUN_ENC2 = 0;

    public static final List<Waypoint> path = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(5.0, 7.0, 0.), new Waypoint(10., 7., Math.PI));
    //public static final List<Waypoint> complicatedPath = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(10.0, 0.0, 0.0), new Waypoint(0.0, 10.0, 0.0));
    public static final List<Waypoint> turnInPlace = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(0.0, 0.0, 90.0));
}