package trajectory_lib;


public class QuinticHermiteSpline implements Spline {
	private double a,b,c,d,e;
	
	private double x_offset, y_offset, angle_offset;
	//Essentially straight line distance between waypoints
	private double knot_distance;
	private double s;
	
	
	public QuinticHermiteSpline(Waypoint start, Waypoint end) {
		x_offset = start.x;
		y_offset = start.y;
		
		double d = Math.sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
		
		knot_distance = d;
		
		angle_offset = Math.atan2(end.y - start.y, end.x - start.x);
		
		double a0_delta = Math.tan(Utils.boundHalfRadians(start.theta - angle_offset));
		double a1_delta = Math.tan(Utils.boundHalfRadians(end.theta - angle_offset));
		
		a = -(3.0 * a0_delta + 3.0 * a1_delta) / (d*d*d*d);
		b = (8.0 * a0_delta + 7.0 * a1_delta) / (d*d*d);
		c = -(6.0 * a0_delta + 4.0 * a1_delta) / (d*d);
		this.d = 0.0;
		e = a0_delta;
	
	}
	

	@Override
	public Coord coords(double percent) {
		double percentage = Math.max(Math.min(percent, 1.0), 0.0);
		
		double x = percentage * knot_distance;
		double y = (a * x + b) * (x*x*x*x) + (c * x + d) * (x*x) + e * x;
		
		double cos_theta = Math.cos(angle_offset);
		double sin_theta = Math.sin(angle_offset);
		
		Coord coord = new Coord();
		coord.x = x * cos_theta - y * sin_theta + x_offset;
		coord.y = x * sin_theta + y * cos_theta + y_offset;
		return coord;
	}

	@Override
	public double deriv(double percent) {
		double percentage = Math.max(Math.min(percent, 1.0), 0.0);
		double x = percentage * knot_distance;
		return (5.0 * a * x + 4.0 * b) * (x*x*x) + (3.0 * c * x + 2.0 * d) * x + e;
	}

	@Override
	public double angle(double percent) {
		return Utils.boundHalfRadians(Math.atan(deriv(percent)) + angle_offset);
	}

	@Override
	public double distance(int samples) {
		double double_samples = (double) samples;
		
		double arc_length = 0.0, t, dydt;
		
		double deriv0 = deriv(0.0);
		
		double integrand;
		double last_integrand = Math.sqrt(1.0 + deriv0 * deriv0) / double_samples;
		
		for(int i = 0; i <= samples; i++) {
			t = ((double)(i)) / double_samples;
			dydt = deriv(t);
			integrand = Math.sqrt(1.0 + dydt * dydt) / double_samples;
			arc_length += (integrand + last_integrand) / 2.0;
			last_integrand = integrand;
		}
		
		double length = knot_distance * arc_length;
		s = length;
		return length;
	}

	@Override
	public double progress_for_distance(double distance, int samples) {
		double double_samples = (double) samples;
		
		double arc_length = 0.0, t = 0.0, dydt, last_arc_length = 0.0;
		
		double deriv0 = deriv(0.0);
		
		double integrand = 0.0;
		double last_integrand = Math.sqrt(1.0 + deriv0 * deriv0) / double_samples;
		
		distance /= knot_distance;
		
		for(int i = 0; i <= samples; i++) {
			t = ((double) (i)) / double_samples;
			dydt = deriv(t);
			integrand = Math.sqrt(1.0 + dydt * dydt) / double_samples;
			arc_length += (integrand + last_integrand) / 2.0;
			if (arc_length > distance) {
				break;
			}
			last_integrand = integrand;
			last_arc_length = arc_length;
		}
		
		double interpolated = t;
		if(arc_length != last_arc_length) {
			interpolated += ((distance - last_arc_length) / 
					(arc_length - last_arc_length) - 1.0) / double_samples;
					
		}
		return interpolated;
	}

}
