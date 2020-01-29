package trajectory_lib;


public class Utils {
	
	/**
     * Convert degrees to radians. This is included here for static imports. In this library, all angle values are
     * given in radians
     */
    public static double d2r(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees. This is included here for static imports.
     */
    public static double r2d(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Bound an angle (in degrees) to -180 to 180 degrees.
     */
    public static double boundHalfDegrees(double angle_degrees) {
    	double angle = angle_degrees;
        while (angle >= 180.0) {
        	angle -= 360.0;
        }
        while (angle < -180.0) {
        	angle += 360.0;
        }
        return angle;
    }
    
    public static double boundHalfRadians(double angle_radians) {
    	double angle = angle_radians;
        while (angle >= Math.PI) {
        	angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
        	angle += 2.0 * Math.PI;
        }
        return angle;
    }
    
    public static double boundRadians(double angle) {
    	double newAngle = angle % (2.0 * Math.PI);
    	if(newAngle < 0) {
    		newAngle = 2.0 * Math.PI + newAngle; 
    	}
    	return newAngle;
    }

}
