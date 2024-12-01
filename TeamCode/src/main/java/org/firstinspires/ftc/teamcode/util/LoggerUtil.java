package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;


import java.util.Set;

public class LoggerUtil {
    private static final String TAG = "Fenix";

    private static boolean isLoggingEnabled = true;
    public static void setIsLoggingEnabled(boolean logEnable){
        isLoggingEnabled = logEnable;
    }
    public static void debug(String message){
        if(isLoggingEnabled) {
            RobotLog.dd(TAG, message);
        }
    }
    public static void verbose(String message){
        if(isLoggingEnabled) {
            RobotLog.vv(TAG, message);
        }
    }
    public static void error(String message){
        if(isLoggingEnabled) {
            RobotLog.ee(TAG, message);
        }
    }
    public static void info(String message){
        if(isLoggingEnabled) {
            RobotLog.ii(TAG, message);
        }
    }
    public static void warning(String message){
        if(isLoggingEnabled) {
            RobotLog.ww(TAG, message);
        }
    }
    public static void logException(Exception e){
        if(isLoggingEnabled) {
            RobotLog.logStackTrace(e);
        }
    }
    public String convertSetToLoggableString(Set set){
        return String.join(",", set);
    }
}
