package org.firstinspires.ftc.teamcode.modules.arm;

public class ElbowIntakeAngleFunction {
    public static double breakingPoint = 3.9;

    public static double getElbowAngle(double slideLength){
        if (slideLength < breakingPoint){
            return -3;
        } else{
            return -1;
        }
    }
}
