package org.firstinspires.ftc.teamcode.subsytems.arm;
import com.arcrobotics.ftclib.controller.PIDController;

public class PIDControl {
    private  PIDController controller;
    private double f;
    private double ticksToUnit;
    public PIDControl(PIDController controller, double p, double i, double d, double f, double ticksToUnit){
        this.controller = controller;
        this.f = f;
        this.ticksToUnit = ticksToUnit;
        controller.setPID(p, i, d);
    }
    public double moveToPos (int currentPos, int targetPos){
        double pid = controller.calculate(currentPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos/ticksToUnit))*f;
        double power = pid + ff;
        return power;
    }
    public void setPID(double kP, double kI, double kD, double f){
        this.f = f;
        controller.setPID(kP, kI, kD);
    }
    public void setPID(double kP, double kI, double kD){
        controller.setPID(kP, kI, kD);
    }
    public void setPID(double kP, double kD){
        controller.setPID(kP, controller.getI(), kD);
    }
    public void setPID(double kP){
        controller.setPID(kP, controller.getI(), controller.getD());
    }
}
