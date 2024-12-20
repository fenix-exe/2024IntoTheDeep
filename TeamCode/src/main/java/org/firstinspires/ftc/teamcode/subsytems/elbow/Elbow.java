package org.firstinspires.ftc.teamcode.subsytems.elbow;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import androidx.annotation.NonNull;

import static java.lang.Math.floor;


public class Elbow {
    public DcMotorEx elbowMotor;
    RevTouchSensor limitSwitch;
    int elbowPosition;
    public int topPosition;
    PIDControl controller;
    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, PIDControl controller, int topPosition){
        this.elbowMotor = elbow;
        this.controller = controller;
        this.topPosition = topPosition;
        this.limitSwitch = limitSwitch;
    }
    private void goToTargetPosition(int targetPosition){
        elbowMotor.setTargetPosition(targetPosition);
        elbowMotor.setPower(1);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setPower(joystickControl);
    }
    public void holdPosition(){
        elbowMotor.setPower(0);
    }
    public double ticksToDegrees(int ticks){
        return ticks/24.22;
    }
    public int degreesToTicks(double degrees){
        return (int) floor(degrees * 24.22);
    }
    public double getElbowAngle(){
        return ticksToDegrees(elbowMotor.getCurrentPosition());
    }

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





}
