package trajectory_lib;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.List;

public class TrajectoryGenerator {
	
	private TrajectoryGenerator() {
		
	}

	//Creates an s-curve profile with quintic hermite spline paths
	public static Trajectory generateQuinticHermiteSpline(TrajectoryConfig c, List<Waypoint> path) {
		if(path.size() < 2) {
			System.out.println("Path is too short, exiting.");
			return null;
		}
		
		double totalDist = 0.0;
		
		QuinticHermiteSpline[] splines = new QuinticHermiteSpline[path.size() - 1];
		double[] arclengths = new double[path.size() - 1];
		
		for(int i = 0; i < path.size() - 1; i++) {
			QuinticHermiteSpline s = new QuinticHermiteSpline(path.get(i), path.get(i + 1));
			double dist = s.distance(c.samples);
			splines[i] = s;
			arclengths[i] = dist;
			totalDist += dist;
		}

		
		double max_a2 = c.max_acc * c.max_acc;
		double max_j2 = c.max_jerk * c.max_jerk;
		
		double trunc_max_v = Math.min(c.max_vel, (-max_a2 + Math.sqrt(max_a2 * max_a2 + 4.0 * max_j2 * c.max_acc * totalDist)) /
				(2 * c.max_jerk));
		
		int filter1 = (int) Math.ceil((trunc_max_v / c.max_acc) / c.dt);
		int filter2 = (int) Math.ceil((c.max_acc / c.max_jerk) / c.dt);
		
		double impulse = (totalDist / trunc_max_v) / c.dt;
		int time = (int) Math.ceil(filter1 + filter2 + impulse);
		
		
		//totalLength = totalDist, path_length = path.size(), trajectory_length = time
		Trajectory traj = secondOrderFilter(filter1, filter2, c.dt, 0.0, trunc_max_v, impulse, time);
		
		/*for(int i = 0; i < time; i++) {
			traj.path[i].theta = path.get(0).theta;
		}*/
		
		int spline_i = 0;
		double spline_init_pos = 0.0, splines_complete = 0.0;
		
		for(int i = 0; i < time; i++) {
			double pos = traj.path[i].pos;
			
			boolean found = false;
			while(!found) {
				double rel_pos = pos - spline_init_pos;
				if(rel_pos <= arclengths[spline_i]) {
					QuinticHermiteSpline s_i = splines[spline_i];
					double percent = s_i.progress_for_distance(rel_pos, c.samples);
					Coord coord = s_i.coords(percent);
					traj.path[i].theta = s_i.angle(percent);
					traj.path[i].x = coord.x;
					traj.path[i].y = coord.y;
					found = true;
				} else if (spline_i < path.size() - 2) {
					splines_complete += arclengths[spline_i];
					spline_init_pos = splines_complete;
					spline_i++;
				} else {
					QuinticHermiteSpline s_i = splines[path.size() - 2];
					traj.path[i].theta = s_i.angle(1.0);
					Coord coord = s_i.coords(1.0);
					traj.path[i].x = coord.x;
					traj.path[i].y = coord.y;
					found = true;
				}
			}
		}
		return traj;
	}
	
	public static AutoTrajectory makeLeftRightTrajectories(Trajectory traj, double wheelbase) {
		Trajectory left = new Trajectory(traj.length());
		Trajectory right = new Trajectory(traj.length());
		double w = wheelbase / 2.0;
		
		for(int i = 0; i < traj.length(); i++) {
			Segment current = traj.get(i);
			left.path[i] = new Segment();
			right.path[i] = new Segment();
			
			left.path[i] = traj.get(i).copy();
			right.path[i] = traj.get(i).copy();
			
			double cos_theta = Math.cos(current.theta);
			double sin_theta = Math.sin(current.theta);
			
			left.path[i].x = current.x - w * sin_theta;
			left.path[i].y = current.y + w * cos_theta;
			
			if(i > 0) {
				Segment last = left.get(i-1);
				double dx = left.path[i].x - last.x;
				double dy = left.path[i].y - last.y;
				double distance = Math.sqrt(dx * dx + dy * dy);
				
				left.path[i].pos = last.pos + distance;
				left.path[i].vel = distance / current.dt;
				left.path[i].acc = (left.path[i].vel - last.vel) / current.dt;
				left.path[i].jerk = (left.path[i].acc - last.acc) / current.dt;
			}
			
			right.path[i].x = current.x + w * sin_theta;
			right.path[i].y = current.y - w * cos_theta;
			
			if(i > 0) {
				Segment last = right.get(i-1);
				double dx = right.path[i].x - last.x;
				double dy = right.path[i].y - last.y;
				double distance = Math.sqrt(dx * dx + dy * dy);
				
				right.path[i].pos = last.pos + distance;
				right.path[i].vel = distance / current.dt;
				right.path[i].acc = (right.path[i].vel - last.vel) / current.dt;
				right.path[i].jerk = (right.path[i].acc - last.acc) / current.dt;
			}
		}
		
		return new AutoTrajectory(left, right);
	}
	
	//You need to write the file with the csv extension. ex: "C:\\Users\\Maverick1\\eclipse-workspace\\TrajectoryGenerator\\src\\Test.csv"
	public static void saveAutoTrajectory(String filePath, AutoTrajectory traj) {
		PrintWriter mOutput = null;
		try {
			mOutput = new PrintWriter(filePath);
			if (mOutput != null) {
				mOutput.println(filePath);
	            mOutput.println(traj.left.length());
	            mOutput.flush();
	        }
		} catch(FileNotFoundException e) {
			e.printStackTrace();
		}
		ReflectingCSVAppender writer = new ReflectingCSVAppender(filePath, true);
		Trajectory left, right;
		left = traj.left;
		right = traj.right;
		for(int i = 0; i < left.length(); i++) {
			writer.add(left.get(i));
		}
		for(int i = 0; i < right.length(); i++) {
			writer.add(right.get(i));
		}
		writer.flush();
	}
	
	public static AutoTrajectory loadAutoTrajectory(String filePath) {
		AutoTrajectoryDeserializer deserializer = new AutoTrajectoryDeserializer(filePath);
		return deserializer.deserialize();
	}
	
	private static Trajectory secondOrderFilter(int filter1, int filter2, double dt, double start_vel, double max_vel,
			double impulse, int length) {
		
		if(length < 0) {
			System.out.println("Error in profile generation: length is negative");
			return null;
		}
		Trajectory traj = new Trajectory(length);
		Segment last = new Segment();
		last.dt = dt;
		last.x = 0.0;
		last.y = 0.0;
		last.pos = 0.0;
		last.vel = start_vel;
		last.acc = 0.0;
		last.jerk = 0.0;
		
		double[] f1 = new double[length];
		f1[0] = (start_vel / max_vel) * filter1; //IUWHEFIUHWEPIFHWEH
		double f2;
		for(int i = 0; i < length; i++) {
			double input = Math.min(impulse, 1.0);
			if(input < 1.0) {
				input -= 1.0;
				impulse = 0.0;
			} else {
				impulse -= input;
			}
			
			double f1_last;
			if(i > 0) {
				f1_last = f1[i-1];
			} else {
				f1_last = f1[0];
			}
			f1[i] = Math.max(0.0, Math.min(filter1, f1_last + input));
			
			f2 = 0.0;
			
			for(int j = 0; j < filter2; j++) {
				if(i - j < 0.0) {
					break;
				}
				
				f2 += f1[i-j];
			}
			f2 = f2 / filter1;
			traj.path[i].vel = f2 / filter2 * max_vel;
			
			traj.path[i].pos = (last.vel + traj.path[i].vel) / 2.0 * dt + last.pos;
			
			traj.path[i].x = traj.path[i].pos;
			traj.path[i].y = 0.0;
			
			traj.path[i].acc = (traj.path[i].vel - last.vel) / dt;
			traj.path[i].jerk = (traj.path[i].acc - last.acc) / dt;
			traj.path[i].dt = dt;
			
			last = traj.path[i];
		}
		
		return traj;
		
	}

}
