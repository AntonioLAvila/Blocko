package trajectory_lib;


public class Trajectory {
	public Segment[] path;
	
	public Trajectory(Segment[] path) {
		this.path = path;
	}
	
	public Trajectory(int length) {
		path = new Segment[length];
		for(int i = 0; i < path.length; i++) {
			path[i] = new Segment();
		}
	}
	
	public Segment get(int i) {
		
		return path[i];
	}
	
	public int length() {
		return path.length;
	}
	
	public Trajectory copy() {
		Trajectory copy = new Trajectory(length());
		for(int i = 0; i < length(); i++) {
			copy.path[i] = get(i).copy();
		}
		return copy;
	}

}
