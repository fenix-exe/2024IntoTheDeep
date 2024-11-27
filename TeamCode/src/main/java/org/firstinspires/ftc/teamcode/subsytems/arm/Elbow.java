package org.firstinspires.ftc.teamcode.subsytems.arm;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import androidx.annotation.NonNull;

import static java.lang.Math.floor;


public class Elbow {
    DcMotorEx elbow;
    RevTouchSensor limitSwitch;
    int elbowPosition;
    int topPosition;
    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, int topPosition){
        this.elbow = elbow;
        this.topPosition = topPosition;
        this.limitSwitch = limitSwitch;
    }
    private void goToTargetPosition(int targetPosition){
        elbow.setTargetPosition(targetPosition);
        elbow.setPower(1);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setTargetAngle(double degrees){
        elbowPosition = degreesToTicks(degrees);
        if (elbowPosition < -140){
            elbowPosition = -140;
        }
        if (elbowPosition > topPosition){
            elbowPosition = topPosition;
        }

        goToTargetPosition(elbowPosition);
    }
    public void elbowJoystick(double joystickControl){
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double power;
        if (joystickControl < 0 && elbow.getCurrentPosition() < -140){
            power = 0;
        } else if (joystickControl > 0 && elbow.getCurrentPosition() > topPosition - 50){
            power = 0;
        } else {
            power = joystickControl;
        }
        elbow.setPower(power);
    }
    public void holdPosition(){
        elbow.setPower(0);
    }
    public void setNewTopPos(int topPosition){
        this.topPosition = topPosition;
    }
    public double ticksToDegrees(int ticks){
        return ticks/24.22;
    }
    public int degreesToTicks(double degrees){
        return (int) floor(degrees * 24.22);
    }
    public double getElbowAngle(){
        return ticksToDegrees(elbow.getCurrentPosition());
    }

    public void setElbowAngle(double angle){
        int ticks = degreesToTicks(angle);
        setTargetAngle(ticks);
    }
    public int getElbowTicks(){
        return elbow.getCurrentPosition();
    }

    public boolean isBusy() {
        return elbow.isBusy();
    }
    public void resetEncoder(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setElbowPower(double power){
        elbow.setPower(power);
    }

    public class elbowControl implements Action {
        private final int targetPos;


        elbowControl(int targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goToTargetPosition(targetPos);
            elbow.setPower(0);
            return true;
        }
    }

    public Action elbowControl(int targetPos){
        return new elbowControl(targetPos);
    }




}
