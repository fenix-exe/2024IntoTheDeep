package org.firstinspires.ftc.teamcode.teleop.modules.arm;

public class ElbowIntakeAngleFunction {
    static double breakingPoint = 7.9;
    static double downIntakePos = -2;
    static double upIntakePos = -4;
    public static double getElbowAngle(double slideLength){
        if (slideLength < breakingPoint){
            return upIntakePos;
        } else {
            return downIntakePos;
        }
    }
}
