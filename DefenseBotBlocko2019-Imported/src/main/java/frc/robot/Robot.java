/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.MjpegServer;

import java.io.OutputStream;
import java.sql.Time;
import java.util.Arrays;
import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.sun.jdi.LongValue;
import com.revrobotics.ColorSensorV3;

import trajectory_lib.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.auto.AutoModeExecuter;
import frc.auto.modes.DoNothingMode;
import frc.auto.modes.JustFollowATrajectory;
import frc.auto.modes.TurnInPlace;
//import edu.wpi.first.wpilibj.Joystick;
import frc.loops.*;

import frc.util.*;

import com.revrobotics.ColorSensorV3;

import frc.subsystems.Drive;
import frc.subsystems.Intake;
import frc.subsystems.LimelightProcessor;
import frc.subsystems.Intake.IntakeState;
//import frc.subsystems.Drive.DriveState;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private AutoModeExecuter autoModeExecuter = new AutoModeExecuter();

  //int x = 2147483647;
  //private LimelightProcessor processor = LimelightProcessor.getInstance();
  private Drive drive = Drive.getInstance();
  private Intake intake = Intake.getInstance();

  boolean autoIntake = false;

  private final SubsystemManager subsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance() /*LimelightProcessor.getInstance() /*Intake.getInstance()*/));

  Looper internalLooper = new Looper();
  
  private static XboxController base = new XboxController(0);
  private static XboxController co = new XboxController(1);

  private static AxisGreater baseRightTrigger = new AxisGreater(base, 3, 0.3);
  private static AxisGreater baseLeftTrigger = new AxisGreater(base, 2, 0.3);

  private static AxisGreater coRightTrigger = new AxisGreater(co, 3, 0.3);
  private static AxisGreater coLeftTrigger = new AxisGreater(co, 2, 0.3);

  private ArcadeDriveHelper arcadeDriveHelper = new ArcadeDriveHelper();
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1.5);
  private SlewRateLimiter turnRateLimiter = new SlewRateLimiter(1.5);

  //private PID pid = new PID(0.004, 0.0, 0.00, -1.0, 1.0);
  
  //Intake.IntakeState shahe = intake.new IntakeState();
  public enum AutoMode{
    DO_NOTHING,
    FOLLOW_PATH,
    TURN_IN_PLACE
  }
  private AutoMode autoMode = AutoMode.DO_NOTHING;

  public Robot() {
    CrashTracker.logRobotConstruction();
  }

  public void zeroSensors() {
    subsystemManager.zeroSensors();
  } 

  public void allPeriodic() {
    subsystemManager.outputToSmartDashboard();
    subsystemManager.writeToLog();
    internalLooper.outputToSmartDashboard();
  }

  
  Intake.IntakeState shahe = intake.new IntakeState();

  public void operatorControls2(){
    //DRIVE
    DriveSignal signal;
    double throttle, turn;
    boolean isHighGear;

    isHighGear = !baseRightTrigger.get();
    throttle = -slewRateLimiter.limit(base.getY(Hand.kLeft));
    turn = turnRateLimiter.limit(base.getX(Hand.kRight));

    signal = arcadeDriveHelper.arcadeDrive(turn, throttle);//throttle, turn

    drive.setHighGear(isHighGear);
    drive.setOpenLoop(signal);
    
    if(base.getBButtonPressed()){
      drive.zeroGyro();
    }
    //INTAKE
    if(co.getAButtonPressed()){
      shahe.isExtended = !intake.getState().isExtended;
    } else{
      shahe.isExtended = intake.getState().isExtended;
    }
  
    if(co.getXButtonPressed()){
      //autoIntake = !autoIntake;
      intake.setAutoRunning(true);
    }
  
    if(intake.getAutoRunning()){
      shahe.autoEject = true;
    }else{
      shahe.autoEject = false;
    }

    intake.setWantedState(shahe);
  }

  //for idiots
  public void operatorControls(){
    //DRIVE
    DriveSignal signal;
    double throttle, turn;
    boolean isHighGear;

    isHighGear = !baseRightTrigger.get();
    throttle = -slewRateLimiter.limit(base.getY(Hand.kLeft));
    turn = turnRateLimiter.limit(base.getX(Hand.kRight));

    signal = arcadeDriveHelper.arcadeDrive(turn, throttle);//throttle, turn

    drive.setHighGear(isHighGear);
    drive.setOpenLoop(signal);

    //INTAKE
    Intake.IntakeState wantedIntakeState = intake.new IntakeState();

    if(co.getAButtonPressed()) {
      wantedIntakeState.isExtended = !intake.getState().isExtended;
      //wantedIntakeState.isExtended = true;
    } else{
      wantedIntakeState.isExtended = intake.getState().isExtended;
      //wantedIntakeState.isExtended = false;
    }

    if(co.getXButtonPressed()){
      wantedIntakeState.isEjected = !intake.getState().isEjected;
    }else{
      wantedIntakeState.isEjected = intake.getState().isEjected;
    }
    intake.setWantedState(wantedIntakeState);

  }


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
   try {
    //processor.setLedMode(3);
      subsystemManager.registerEnabledLoops(internalLooper);
      // for usb cam
      // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      // camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
      // MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", 0);
      // cameraServer.setSource(camera);
			// camera.setResolution(640, 480);
      // camera.setFPS(30);
      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
			throw t;
    }
    zeroSensors();
  }

  @Override
  public void disabledInit() {
    try {
      //table.getEntry("ledMode").setNumber(3);

      CrashTracker.logDisabledInit();
			internalLooper.stop();
			subsystemManager.stop();
      drive.setOpenLoop(DriveSignal.NEUTRAL);
      slewRateLimiter.reset();
      turnRateLimiter.reset();
      drive.setBrakeMode(false);
      if(autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void disabledPeriodic() {
    allPeriodic();
    //drive.setOpenLoop(DriveSignal.NEUTRAL);
    System.out.println(" tx:"+table.getEntry("tx").getDouble(100));//grab network table val for skew
    if(base.getAButtonPressed()){
      autoMode = AutoMode.FOLLOW_PATH;
    }
    if(base.getBButtonPressed()){
      autoMode = AutoMode.TURN_IN_PLACE;
    }
    //System.out.println(autoMode);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */


  @Override
  public void autonomousInit() {
    try{
      zeroSensors();
      internalLooper.start();
      CrashTracker.logAutoInit();
      System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

      autoModeExecuter = new AutoModeExecuter();

      switch(autoMode){
        case DO_NOTHING:
          autoModeExecuter.setAutoMode(new DoNothingMode());
          break;
        case FOLLOW_PATH:
          autoModeExecuter.setAutoMode(new JustFollowATrajectory());
          System.out.println("follow path");
          break;
        case TURN_IN_PLACE:
          autoModeExecuter.setAutoMode(new TurnInPlace());
          System.out.println("TIP case");
          break;
        default:
          System.out.println("you suck do it better next time, running do nothing");
          autoModeExecuter.setAutoMode(new DoNothingMode());
      }

      autoModeExecuter.start();

    } catch(Throwable t){
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //operatorControls2();
    allPeriodic();
    
  }

  @Override
  public void teleopInit() {
    try {
			CrashTracker.logTeleopInit();
			internalLooper.start();
		  drive.setOpenLoop(DriveSignal.NEUTRAL);
			drive.setBrakeMode(false);
      slewRateLimiter.reset();
      turnRateLimiter.reset();
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
    operatorControls2();
    System.out.println(drive.getHeading());
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
    }
    allPeriodic();
  }

@Override
public void testInit() {  
  internalLooper.start();
  allPeriodic();
  zeroSensors();
  pid.reset();

  table.getEntry("ledMode").setNumber(3);

  // Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(10.0, 5.0, 0.0)));
  // AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
  // drive.setTrajectory(traj, false);
}
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /**
   * This function is called periodically during test mode.
   */
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private PID pid = new PID(0.02, 0.0, 0.00, -1.0, 1.0);

@Override
public void testPeriodic() {
  //drive.startPathFollowing();
  //colorSensor.getColor();
  //System.out.println(table.getEntry("tx").getDouble(0.0));
  //System.out.println(drive.getHeading());
  double isDetected = table.getEntry("tv").getDouble(0);

  if(base.getAButton() && isDetected == 1){
    double val = pid.calculate(table.getEntry("tx").getDouble(0.0), 0.,Constants.LOOPER_DT);
    DriveSignal signal = new DriveSignal(-val, -val);
    drive.setOpenLoop(signal);
  }else if(base.getAButton() && isDetected == 0){
    drive.setOpenLoop(new DriveSignal(.4, .4));
  }else{
    drive.setOpenLoop(DriveSignal.NEUTRAL);
  }
  
  }
}
