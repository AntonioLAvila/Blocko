package trajectory_lib;


public class Segment {
	
	public double dt, x, y, pos, vel, acc, jerk, theta;
	
	public Segment() {
		
	}
	
	public Segment(double dt, double x, double y, double pos, double vel, double acc, double jerk, double theta) {
		this.dt = dt;
		this.x = x;
		this.y = y;
		this.pos = pos;
		this.vel = vel;
		this.acc = acc;
		this.jerk = jerk;
		this.theta = theta;
	}
	
	public Segment copy() {
		return new Segment(dt, x, y, pos, vel, acc, jerk, theta);
	}
	
	public boolean equals(Segment other) {
		return other.dt == dt && other.x == x && other.y == y &&
				other.pos == pos && other.vel == vel && other.acc == acc &&
				other.jerk == jerk && other.theta == theta;
	}

}
