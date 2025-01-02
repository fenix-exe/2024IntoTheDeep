package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;


import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class LoggerUtil {
    private static final String TAG = "Fenix";

    private static final String logFileName = "/sdcard/Download/logs/logs.tsv";

    private static boolean isLoggingEnabled = true;
    private static boolean writeToFile = true;

    private static PrintWriter stream = null;

    public static void setIsLoggingEnabled(boolean logEnable){
        isLoggingEnabled = logEnable;
    }
    public static void setWriteToFile(boolean writeToFile){LoggerUtil.writeToFile = writeToFile;}
    public static void debug(String type, String message){
        if(isLoggingEnabled) {
            RobotLog.dd(TAG + ":" + type, message);
            if(writeToFile){
                printToLogFile(TAG + ":" + type, "debug", message);
            }
        }
    }
    public static void verbose(String type, String message){
        if(isLoggingEnabled) {
            RobotLog.vv(TAG + ":" + type, message);
            if(writeToFile){
                printToLogFile(TAG + ":" + type, "verbose", message);
            }
        }
    }
    public static void error(String type, String message){
        if(isLoggingEnabled) {
            RobotLog.ee(TAG + ":" + type, message);
            if(writeToFile){
                printToLogFile(TAG + ":" + type,"error", message);
            }
        }
    }
    public static void info(String type, String message){
        if(isLoggingEnabled) {
            RobotLog.ii(TAG + ":" + type, message);
            if(writeToFile){
                printToLogFile(TAG + ":" + type,"info", message);
            }
        }
    }
    public static void warning(String type, String message){
        if(isLoggingEnabled) {
            RobotLog.ww(TAG + ":", message);
            if(writeToFile){
                printToLogFile(TAG + ":" + type,"warning", message);
            }
        }
    }
    public static void logException(String type, Exception e){
        if(isLoggingEnabled) {
            RobotLog.logStackTrace(e);
            if(writeToFile){
                printToLogFile(TAG + ":" + type,"debug", e.getMessage());
            }
        }
    }
    private static void printToLogFile(String tag, String typeOfMessage, String message){
        try{
            if (stream == null){
                File file = new File(logFileName);
                file.getParentFile().mkdirs();
                file.createNewFile();
                stream = new PrintWriter(new FileOutputStream(file, false), true);
            }
            //writing logs
            String timeStamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss:SSS").format(Calendar.getInstance().getTime());
            stream.println(timeStamp + "\t" + tag + "\t" + typeOfMessage + "\t" + message);
        } catch (Exception e){
            RobotLog.logStackTrace(e);
            //LoggerUtil.logException(e);
        }
    }
    public static void logFlush(){
        if (stream != null){
            stream.close();
        }
    }
}
