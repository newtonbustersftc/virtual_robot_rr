package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.Buffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

// Written by Lucas
// Oct. 12, 2019

public class Logger {
    static PrintWriter pw;

    public static void init() {
        try {
            folderCleanup("driver", ".txt", 30);
            folderCleanup("S", ".jpg", 20);
            String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
            File file = new File("/Users/haifeng/IdeaProjects/virtual_robot_rr/data/driver_" + timestamp + ".txt");

            pw = new PrintWriter(new BufferedWriter(new FileWriter(file)));
        } catch(IOException e) {
            throw new RuntimeException("cannot write to file", e);
        }
    }
    public static void logFile(String message) {
        try {
            if (pw == null) {
                init();
            }
            String timestamp = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date());

            pw.println(timestamp + " " + message);
            pw.flush();     // for now
        } catch (Exception e) {
            throw new RuntimeException("cannot write to file");
        }
    }

    public static void flushToFile()
    {
        try {
            pw.flush();
        } catch(Exception e) {
            throw new RuntimeException("cannot flush");
        }
    }

    /** Delete files more than 30 minutes old */
    public static void folderCleanup(String startsWith, String endsWith, int maxCnt) {
        // could use wildcardfilter, but not want to include another Library
        File file = new File("/Users/haifeng/IdeaProjects/virtual_robot_rr/data");
        ArrayList<File> fileList = new ArrayList<File>();
        for(File f : file.listFiles()) {
            if (f.isFile() && f.getName().startsWith(startsWith) && f.getName().endsWith(endsWith)) {
                if (f.lastModified() < System.currentTimeMillis()-1800000) {
                    RobotLog.i("Deleting " + f.getName());
                    f.delete();
                }
                else {
                    fileList.add(f);
                }
            }
        }
        int extra = fileList.size() - maxCnt;
        for(int i=0; i<extra; i++) {
            File f = fileList.remove(0);
            RobotLog.i("Deleting " + f.getName());
            f.delete();
        }
    }
}
