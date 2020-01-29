package trajectory_lib;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.StringTokenizer;

public class AutoTrajectoryDeserializer {
	BufferedReader fileString = null;
	String file = "";
	Charset charset = Charset.forName("ISO-8859-1");
	
  public AutoTrajectoryDeserializer(String csvFile) {
	 Path path = Paths.get(csvFile);
	try {
		List<String> lines = Files.readAllLines(path, charset);
		file += csvFile;
		for (String line : lines) {
	        file += line + "\n";
		}
		//System.out.println(file);
	} catch (FileNotFoundException e) {
		e.printStackTrace();
	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	if(fileString != null) {
		for(int i = 0; i < fileString.lines().count(); i++) {
			try {
				System.out.println(file);
				file += fileString.readLine() + "\n";
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		System.out.println(file);
	}
  }

  public AutoTrajectory deserialize() {
    StringTokenizer tokenizer = new StringTokenizer(file, "\n");
    System.out.println("Parsing path string...");
    System.out.println("String has " + file.length() + " chars");
    System.out.println("Found " + tokenizer.countTokens() + " tokens");
    
    String name = tokenizer.nextToken(); 
	  
    int num_elements = Integer.parseInt(tokenizer.nextToken());
    
    Trajectory left = new Trajectory(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      Segment segment = new Segment();
      StringTokenizer line_tokenizer = new StringTokenizer(
              tokenizer.nextToken(), " ");
      
      segment.dt = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.x = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.y = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.pos = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.vel = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.acc = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.jerk = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.theta = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      
      left.path[i] = segment;
    }
    Trajectory right = new Trajectory(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      Segment segment = new Segment();
      StringTokenizer line_tokenizer = new StringTokenizer(
              tokenizer.nextToken(), " ");
      
      segment.dt = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.x = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.y = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.pos = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.vel = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.acc = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.jerk = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      segment.theta = FastParser.parseFormattedDouble(line_tokenizer.nextToken());
      
      right.path[i] = segment;
    }
    
    System.out.println("...finished parsing path from string.");
    return new AutoTrajectory(left, right);
  }
  
}
