package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import drivers.LazyTalonSRX;
import drivers.LazyVictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.util.DriveSignal;
import frc.util.Util;
import trajectory_lib.*;


public class Drive extends Subsystem {

    private static Drive instance;

    public static Drive getInstance() {
        if(instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public enum DriveState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        TURN_IN_PLACE
    }

    private TalonSRX leftFront, rightFront;
    private VictorSPX leftMid, rightMid, rightRear, leftRear;
    private DoubleSolenoid shifter;
    private PigeonIMU imu;
    //private Encoder leftEnc, rightEnc;

    private boolean isHighGear;
    private boolean isBrakeMode;
    private double angleOffset = 0.;

    //control: path following
    private TrajectoryFollower trajectoryFollower;
    private TrajectoryConfig config, testConfig;
    private boolean reversed = false;

    //control: turn in place
    private double absAngleSetpoint;
    private HeadingController headingController;

    //state
    private DriveState driveState = DriveState.OPEN_LOOP;

    private Drive() {
        // leftFront = TalonSRXFactory.createDefaultTalon(Constants.LEFT_FRONT_DRIVE_TALON);
        // leftMid = VictorSPXFactory.createPermanentSlaveVictor(Constants.LEFT_MID_DRIVE_VICTOR, Constants.LEFT_FRONT_DRIVE_TALON);
        // leftRear = VictorSPXFactory.createPermanentSlaveVictor(Constants.LEFT_REAR_DRIVE_VICTOR, Constants.LEFT_FRONT_DRIVE_TALON);

        // rightFront = TalonSRXFactory.createDefaultTalon(Constants.RIGHT_FRONT_DRIVE_TALON);
        // rightMid = VictorSPXFactory.createPermanentSlaveVictor(Constants.RIGHT_MID_DRIVE_VICTOR, Constants.RIGHT_FRONT_DRIVE_TALON);
        // rightRear = VictorSPXFactory.createPermanentSlaveVictor(Constants.RIGHT_REAR_DRIVE_VICTOR, Constants.RIGHT_FRONT_DRIVE_TALON);
        absAngleSetpoint = 0.0;

        imu = new PigeonIMU(Constants.DRIVE_IMU);


        leftFront = new LazyTalonSRX(Constants.LEFT_FRONT_DRIVE_TALON);
        leftMid = new LazyVictorSPX(Constants.LEFT_MID_DRIVE_VICTOR);
        leftMid.set(ControlMode.Follower, Constants.LEFT_FRONT_DRIVE_TALON);
        leftRear = new LazyVictorSPX(Constants.LEFT_REAR_DRIVE_VICTOR);
        leftRear.set(ControlMode.Follower, Constants.LEFT_FRONT_DRIVE_TALON);

        rightFront = new LazyTalonSRX(Constants.RIGHT_FRONT_DRIVE_TALON);
        rightMid = new LazyVictorSPX(Constants.RIGHT_MID_DRIVE_VICTOR);
        rightMid.set(ControlMode.Follower, Constants.RIGHT_FRONT_DRIVE_TALON);
        rightRear = new LazyVictorSPX(Constants.RIGHT_REAR_DRIVE_VICTOR);
        rightRear.set(ControlMode.Follower, Constants.RIGHT_FRONT_DRIVE_TALON);

        leftFront.enableVoltageCompensation(true);
        rightFront.enableVoltageCompensation(true);

        leftFront.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);

        leftFront.configOpenloopRamp(0.2, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(0.2, Constants.LONG_CAN_TIMEOUT_MS);

        //leftEnc.setDistancePerPulse(Constants.WHEEL_DIAMETER * Math.PI / 360.0);
        // rightEnc.setDistancePerPulse(Constants.WHEEL_DIAMETER * Math.PI / 360.0);
        
        
        leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
        //---------------------------------- test tuff
        trajectoryFollower = new TrajectoryFollower(Constants.TRAJECTORY_FOLLOWER_KP, Constants.TRAJECTORY_FOLLOWER_KI, Constants.HEADING_CONTROLLER_KD, Constants.TRAJECTORY_FOLLOWER_KV,Constants.TRAJECTORY_FOLLOWER_KA, Constants.TRAJECTORY_FOLLOWER_KW);

        config = new TrajectoryConfig(TrajectoryConfig.SAMPLES_FAST, 0.005, Constants.DRIVE_ALLOWED_SPEED, Constants.DRIVE_MAX_ACC, Constants.DRIVE_MAX_JERK);

        trajectoryFollower.getHeadingController().setInputRange(-Math.PI, Math.PI);
        trajectoryFollower.getHeadingController().setPID(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI, Constants.HEADING_CONTROLLER_KD);

        headingController = new HeadingController(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI, Constants.HEADING_CONTROLLER_KD);
        //----------------------------------- end of test stuff

        shifter = new DoubleSolenoid(Constants.SHIFTER_SOLENOID_1, Constants.SHIFTER_SOLENOID_2);

        isHighGear = true;

        isBrakeMode = true;
    }

    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    
        @Override
        public void onStart(double timestamp) {
            zeroEncoders();
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Drive.this)  {
                switch(driveState) {
                    case OPEN_LOOP:
                        return;
                    case PATH_FOLLOWING:
                        handlePathFollowing();
                        return;
                    case TURN_IN_PLACE:
                        handleTurnInPlace();
                        return;
                    default:
                        System.out.println("Unexpected Drive Control State: " + driveState);
                }
            }
        }
    };

//------------------------------------------------------------ test stuff
    public TrajectoryConfig getConfig(){
        return config;
    }

    public boolean pathIsFinished() {
        return trajectoryFollower.isFinished();
    }

    public void setTrajectory(AutoTrajectory traj, boolean reversed) {
        trajectoryFollower.setTrajectory(traj);
        this.reversed = reversed;
    }

    public synchronized void startPathFollowing() {
        if(driveState != DriveState.PATH_FOLLOWING) {
            driveState = DriveState.PATH_FOLLOWING;
            trajectoryFollower.start(getLeftEncoder(), getRightEncoder(), reversed);
        }
    }

    private void handlePathFollowing() {
        DriveSignal signal = trajectoryFollower.calculate(getLeftEncoder(), getLeftEncoderRate(), getRightEncoder(), getRightEncoderRate(), getHeading(), Constants.LOOPER_DT);
        DriveSignal signalToMotor = new DriveSignal(-signal.getLeft(), signal.getRight()); //making sure it goes straight
        
        setMotors(signalToMotor);
        //System.out.println("left: " + signalToMotor.getLeft() + "  right: " + signalToMotor.getRight());
    }

    public synchronized void setAbsoluteAngle(double angleToSet) {
        absAngleSetpoint = angleToSet;
        headingController.reset();
    }

    public synchronized void setRelativeAngle(double relativeAngle) {
        absAngleSetpoint = getHeading() + relativeAngle;
        headingController.reset();
    }

    public synchronized double getAbsAngleSetpoint() {
        return absAngleSetpoint;
    }

    private void handleTurnInPlace() {
        double powerToTurn = headingController.calculate(Utils.boundHalfDegrees(absAngleSetpoint), Utils.boundHalfDegrees(getHeading()), Constants.LOOPER_DT);
        DriveSignal signal = new DriveSignal(powerToTurn, powerToTurn); 
        if(Util.epsilonEquals(getHeading(), absAngleSetpoint, 2.0)) {
            signal = DriveSignal.NEUTRAL;
            //SmartDashboard.putBoolean("Aim Finished", true);
        } else {
            //SmartDashboard.putBoolean("Aim Finished", false);
            setMotors(signal);
        }
        
    }
//----------------------------------------------------------- end of test stuff

    public synchronized void setRampRate(double rampRate) {
        leftFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
    }

    public synchronized void setDriveState(DriveState wantedState) {
        if(driveState != wantedState) {
            driveState = wantedState;
        }
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void setMotors(DriveSignal signal) {
        leftFront.set(ControlMode.PercentOutput, signal.getLeft());
        rightFront.set(ControlMode.PercentOutput, signal.getRight());
    }

    public void ghettoSetMotors(double left, double right){
        leftFront.set(ControlMode.PercentOutput, left);
        rightFront.set(ControlMode.PercentOutput, right);
    }

    //-----------------------------------------------motion magic test yeah right 11/22/19
    public void magicAuto(){

        leftFront.set(ControlMode.MotionMagic, 3);
        rightFront.set(ControlMode.MotionMagic, 3);
    }

    public synchronized void setHighGear(boolean on) {
        if(isHighGear != on) {
            isHighGear = on;
            if(on) {
                shifter.set(Value.kForward);
            } else {
                shifter.set(Value.kReverse);
            }
        }
    }

    public synchronized void setBrakeMode(boolean on) {
        if(isBrakeMode != on) {
            isBrakeMode = on;
            if(on) {
                leftFront.setNeutralMode(NeutralMode.Brake);
                leftRear.setNeutralMode(NeutralMode.Brake);

                rightFront.setNeutralMode(NeutralMode.Brake);
                rightRear.setNeutralMode(NeutralMode.Brake);
            } else {
                leftFront.setNeutralMode(NeutralMode.Coast);
                leftRear.setNeutralMode(NeutralMode.Coast);

                rightFront.setNeutralMode(NeutralMode.Coast);
                rightRear.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if(driveState != DriveState.OPEN_LOOP) {
            driveState = DriveState.OPEN_LOOP;
            setBrakeMode(false);
        }
        setMotors(signal);
    }


    public double getLeftEncoder() { //ft
        return (leftFront.getSelectedSensorPosition() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12)) * -1;
        //return ((leftFront.getSelectedSensorPosition() * (Constants.ENCODER_CONVERSION) * ((Constants.WHEEL_DIAMETER* Math.PI)/360))/12)*-1;
    }

    public double getRightEncoder() {
        return (rightFront.getSelectedSensorPosition() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12)) * -1;
        //return ((rightFront.getSelectedSensorPosition() * (Constants.ENCODER_CONVERSION) * ((Constants.WHEEL_DIAMETER* Math.PI)/360))/12)*-1;
    }


    public double getLeftEncoderRate() {
        return (((leftFront.getSelectedSensorVelocity() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12)))*10) * -1;
    }
    public double getRightEncoderRate() {
        return (((rightFront.getSelectedSensorVelocity() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12)))*10) * -1;
    }

    //gyro
    public void zeroGyro() {
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        angleOffset = imu.getFusedHeading(fusionStatus);
    }

    public double getHeading() {
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        return imu.getFusedHeading(fusionStatus) - angleOffset;
    }

    public double getHeadingRate() {
        double[] rpy = new double[3];
		imu.getRawGyro(rpy);
        return rpy[2];
    }

    public void zeroEncoders(){
        rightFront.setSelectedSensorPosition(0);
        leftFront.setSelectedSensorPosition(0);
        rightRear.setSelectedSensorPosition(0);
        leftRear.setSelectedSensorPosition(0);
    }

    public void writeToLog() {
    }
    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("right pos", getRightEncoder());
        SmartDashboard.putNumber("left pos", getLeftEncoder());
        SmartDashboard.putNumber("right vel", getRightEncoderRate());
        SmartDashboard.putNumber("left vel", getLeftEncoderRate());
        SmartDashboard.putNumber("gyro", getHeading());
    }
    public void stop(){
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    public void zeroSensors(){
        zeroEncoders();
        zeroGyro();
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}