package org.firstinspires.ftc.teamcode.common;

import static java.lang.Math.floor;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class CommonElbow {
    public DcMotorEx elbowMotor;
    public RevTouchSensor limitSwitch;
    int elbowPosition;
    public final double OFFSET = 1;
    public int topPosition;
    public CommonElbow(DcMotorEx elbow, RevTouchSensor limitSwitch, int topPositionInDegrees){
        this.elbowMotor = elbow;
        this.topPosition = degreesToTicks(topPositionInDegrees);
        this.limitSwitch = limitSwitch;
    }
    protected void goToTargetPosition(int targetPosition){
        elbowMotor.setTargetPosition(targetPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(1);
    }
    public void setTargetAngle(double degrees){
        elbowPosition = degreesToTicks(degrees - OFFSET);
        goToTargetPosition(elbowPosition);
    }
    public void setTargetAngleAndSpeed(double deg, double power) {
        elbowPosition = degreesToTicks(deg - OFFSET);
        elbowMotor.setTargetPosition(elbowPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(power);
    }
    public void holdPosition(){
        //this only works because we have a worm gear and we don't want to waste uneccesary battery power
        elbowMotor.setPower(0);
        //goToTargetPosition(elbowMotor.getCurrentPosition());
    }
    public double ticksToDegrees(int ticks){
        return ticks/11.28556;
    }
    public int degreesToTicks(double degrees){
        return (int) floor(degrees * 11.28556);
    }
    public double getElbowAngle(){
        return ticksToDegrees(elbowMotor.getCurrentPosition());
    }
    public double getElbowTargetAngle(){return ticksToDegrees(elbowMotor.getTargetPosition()) + OFFSET;}

    public int getElbowTicks(){
        return elbowMotor.getCurrentPosition();
    }

    public void resetEncoder(){
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setElbowPower(double power){
        elbowMotor.setPower(power);
    }
    public boolean isLimitSwitchPressed(){
        return limitSwitch.isPressed();
    }
}
