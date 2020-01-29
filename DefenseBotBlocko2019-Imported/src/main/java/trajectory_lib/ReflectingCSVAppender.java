package trajectory_lib;


import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.text.DecimalFormat;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file
 */
public class ReflectingCSVAppender {
    ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
    FileWriter mOutput = null;
    Field[] mFields;
    DecimalFormat fmt = new DecimalFormat("#0.000");

    public ReflectingCSVAppender(String fileName, boolean append) {
        mFields = Segment.class.getFields();
        try {
            mOutput = new FileWriter(fileName, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        // Write field names.
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(" ");
            }
            line.append(field.getName());
        }
        writeLine(line.toString());
    }

    public void add(Segment value) {
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(" ");
            }
            try {
            	
                line.append(fmt.format(field.get(value)).toString());
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        mLinesToWrite.add(line.toString());
    }

    protected synchronized void writeLine(String line) {
        if (mOutput != null) {
        	try {
        		mOutput.write(line + "\r\n");
        	} catch(IOException e) {
        		e.printStackTrace();
        	}
        }
    }

    // Call this periodically from any thread to write to disk.
    public void write() {
        while (true) {
            String val = mLinesToWrite.pollFirst();
            if (val == null) {
                break;
            }
            writeLine(val);
        }
    }

    public synchronized void flush() {
        if (mOutput != null) {
            write();
            try {
            	mOutput.flush();
            } catch(IOException e) {
            	e.printStackTrace();
            }
        }
    }
}