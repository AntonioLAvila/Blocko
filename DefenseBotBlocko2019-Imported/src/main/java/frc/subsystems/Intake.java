package frc.subsystems;

import frc.loops.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Intake extends Subsystem {

    private static Intake instance;
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private DoubleSolenoid intake;
    private Solenoid ejector;

    IntakeState wantedState = new IntakeState();
    IntakeState state = new IntakeState();
    private double panelEjectorTimestamp = 0.0;
    private boolean autoRunning = false;

    private Intake() {
        intake = new DoubleSolenoid(Constants.INTAKE_SOL_1, Constants.INTAKE_SOL_2);
        ejector = new Solenoid(Constants.EJECT_SOL);
    }

    public class IntakeState {
        public boolean isExtended = false;
        public boolean isEjected = false;
        public boolean autoEject = false;
    }

    public void handleIntake(IntakeState wanted, IntakeState current, double timestamp) {
        extendIntake(wanted.isExtended);
        eject(wanted.isEjected);
        autoEject(wanted.autoEject, timestamp);
    }

    public IntakeState getState(){
        return state;
    }


    public void extendIntake(boolean extend) {
        if(extend != state.isExtended) {
            state.isExtended = extend;
            if(extend) {
                intake.set(Value.kForward);
            }else{
                intake.set(Value.kReverse);
            }
        }
    }

    public void eject(boolean eject){
        if(eject != state.isEjected){
            state.isEjected = eject;
            ejector.set(eject);
        }
    }
    
    //please use this as an example on how to make a good sequence it worked when the pistons were actually installed.
    /**
     * Auto Ejecting sequence that is a good simlple example for future reference.
     * @param autoEject if you want to auto eject or not
     * @param timestamp to measure time for push in and retract
     */
    public void autoEject(boolean autoEject, double timestamp){
        if(autoEject != state.autoEject){
            state.autoEject = autoEject;
            if(autoEject) {
                wantedState.isExtended = false;
                panelEjectorTimestamp = timestamp;
            }
        }
        if(autoEject){
            System.out.println(timestamp - panelEjectorTimestamp);
            /*if(Math.abs(timestamp - panelEjectorTimestamp) < 0.3){
               //this is for wait
            }else*/ 
            if(Math.abs(timestamp - panelEjectorTimestamp) < 0.3){
                if(wantedState.isEjected != true){
                    wantedState.isEjected = true;
                }
            }else {
                wantedState.isEjected = false;
                autoRunning = false;
            }
        }
    }
    

    public boolean getAutoRunning(){
        return autoRunning;
    }
    public void setAutoRunning(boolean autoRunning){
        this.autoRunning = autoRunning;
    }


    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    
        @Override
        public void onStart(double timestamp) {
            extendIntake(false);//hre
            eject(false);
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Intake.this) {
               handleIntake(wantedState, state, timestamp);
            }
        }
    };
    public synchronized void setWantedState(IntakeState wanted) {
        wantedState = wanted;
    }

    public void writeToLog() {
    };
    public void outputToSmartDashboard() {
    }
    public void stop() {
        intake.set(DoubleSolenoid.Value.kReverse);
        ejector.set(false);
    }
    public void zeroSensors() {
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}