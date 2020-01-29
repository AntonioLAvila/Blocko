package trajectory_lib;


public interface Spline {
	
	public Coord coords(double percent);
	
	public double deriv(double percent);
	
	public double angle(double percent);
	
	public double distance(int samples);
	
	public double progress_for_distance(double distance, int samples);
	
	
}
