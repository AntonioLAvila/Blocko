package trajectory_lib;


public class Waypoint {
	public Waypoint() {
		
	}
	//radians
	public Waypoint(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	
	public double x, y, theta;
}
