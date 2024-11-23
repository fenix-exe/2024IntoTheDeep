package org.firstinspires.ftc.teamcode.subsytems.arm;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import androidx.annotation.NonNull;

import static java.lang.Math.floor;


public class Elbow {
    DcMotorEx elbow;
    RevTouchSensor limitSwitch;
    int elbowPosition;
    int topPosition;
    int STEP_SIZE_FOR_JOYSTICK = 350;
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
        if (elbowPosition < 0){
            elbowPosition = 0;
        }
        if (elbowPosition > topPosition){
            elbowPosition = topPosition;
        }

        goToTargetPosition(elbowPosition);
    }
    public void elbowJoystick(double joystickControl){
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setPower(joystickControl);

        /*int elbowPosition = elbow.getCurrentPosition() + (int) (STEP_SIZE_FOR_JOYSTICK * joystickControl);
        if (elbowPosition > topPosition-50){
            elbowPosition = topPosition;
        }
        if (elbowPosition < -15){
            elbowPosition = -15;
        }
        goToTargetPosition(elbowPosition);*/
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
