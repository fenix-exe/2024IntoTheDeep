package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
//Code is from KookyBotz
public class PivotPIDFFunctions{
    private PIDController controller;
    private static double f;
    private final double ticks_in_degrees = 5281.1/180;
    public PivotPIDFFunctions(PIDController controller, double f){
        this.controller = controller;
        this.f = f;
        controller.setPID(controller.getP(), controller.getI(), controller.getD());
    }
    public double moveToPos (int currentPos, int targetPos){
        double pid = controller.calculate(currentPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos/ticks_in_degrees))*f;
        double power = pid + ff;
        return power;
    }
}