package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;


import java.util.Collections;
import java.util.Set;
import java.util.Vector;

public class loggerUtil{
    private static final String TAG = "Fenix";
    public static void debug(String message){
        RobotLog.dd(TAG, message);
    }
    public static void verbose(String message){
        RobotLog.vv(TAG, message);
    }
    public static void error(String message){
        RobotLog.ee(TAG, message);
    }
    public static void info(String message){
        RobotLog.ii(TAG, message);
    }
    public static void warning(String message){
        RobotLog.ww(TAG, message);
    }
    public static void logException(Exception e){
        RobotLog.logStackTrace(e);
    }
    public String convertSetToLoggableString(Set set){
        return String.join(",", set);
    }
}
