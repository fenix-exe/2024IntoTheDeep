package org.firstinspires.ftc.teamcode.teleop.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double SLIDE_TOLERANCE = 1;
    public static double LOW_SLIDE_TOLERANCE = 0.5;
    public static double ELBOW_TOLERANCE = 3;
    public static double LOW_ELBOW_TOLERANCE = 1;
    public static double LINEAR_ACTUATOR_TOLERANCE = 0.5;
    public static double PHYSICAL_MAX_EXTENSION_IN_INCHES = 30.7;
    public static double OPEN_POSITION = 0.2534;
    public static double CLOSED_POSITION = 0.6028;
    public static double INTERMEDIATE_POSITION = 0.4506;
    public static double NORMAL_SPEED = 0.4;
    public static double SLOW_SPEED = NORMAL_SPEED/2;
    public static double INTAKE_SPEED =0.5;
    public static double OUTTAKE_SPEED =-0.25;
    public static double SLOW_OUTTAKE_SPEED = -1;
    public static double STOP_SPEED = 0;

}
