package frc.util;

public class SlewRateLimiter {
    private double limitedCurrentJoystickValue = 0.0;
    private double limit = 0.2;

    public SlewRateLimiter(double limit) {
        this.limit = limit;
    } 

    public SlewRateLimiter() {

    }

    public double limit(double joystickValue) {
        double change = joystickValue - limitedCurrentJoystickValue;
        if(change > limit) {
            change = limit;
        } else if(change < -limit) {
            change = -limit;
        }
        limitedCurrentJoystickValue += change;
        //System.out.println(change);
        return limitedCurrentJoystickValue;
    }

    public void reset() {
        limitedCurrentJoystickValue = 0.0;
    }


}