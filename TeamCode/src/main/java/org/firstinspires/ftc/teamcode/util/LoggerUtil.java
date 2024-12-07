package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.subsytems.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorPresetPosition;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.util.Calendar;
import java.util.Set;
import java.util.logging.Logger;

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
    public static void debug(String message){
        if(isLoggingEnabled) {
            RobotLog.dd(TAG, message);
            if(writeToFile){
                printToLogFile("debug", message);
            }
        }
    }
    public static void verbose(String message){
        if(isLoggingEnabled) {
            RobotLog.vv(TAG, message);
            if(writeToFile){
                printToLogFile("verbose", message);
            }
        }
    }
    public static void error(String message){
        if(isLoggingEnabled) {
            RobotLog.ee(TAG, message);
            if(writeToFile){
                printToLogFile("error", message);
            }
        }
    }
    public static void info(String message){
        if(isLoggingEnabled) {
            RobotLog.ii(TAG, message);
            if(writeToFile){
                printToLogFile("info", message);
            }
        }
    }
    public static void warning(String message){
        if(isLoggingEnabled) {
            RobotLog.ww(TAG, message);
            if(writeToFile){
                printToLogFile("warning", message);
            }
        }
    }
    public static void logException(Exception e){
        if(isLoggingEnabled) {
            RobotLog.logStackTrace(e);
            if(writeToFile){
                printToLogFile("debug", e.getMessage());
            }
        }
    }
    private static void printToLogFile(String typeOfMessage, String message){
        try{
            if (stream == null){
                File file = new File(logFileName);
                file.getParentFile().mkdirs();
                file.createNewFile();
                stream = new PrintWriter(new FileOutputStream(file, false), true);
            }
            //writing logs
            String timeStamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss:SSS").format(Calendar.getInstance().getTime());
            stream.println(timeStamp + "    " + typeOfMessage + "   " + message);
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
