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
    public RevTouchSensor limitSwitch;
    int elbowPosition;
    public int topPosition;
    PIDControl controller;
    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, PIDControl controller, int topPosition){
        this.elbowMotor = elbow;
        this.controller = controller;
        this.topPosition = topPosition;
        this.limitSwitch = limitSwitch;
    }

    public void goTo(int targetPosition, double speed){
        elbowMotor.setTargetPosition(targetPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(speed);
    }

    private void goToTargetPosition(int targetPosition){
        elbowMotor.setTargetPosition(targetPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(1);
    }
    public void setTargetAngle(double degrees){
        elbowPosition = degreesToTicks(degrees);
        goToTargetPosition(elbowPosition);
    }
    public void setTargetAngleAndSpeed(double deg, double power) {
        elbowPosition = degreesToTicks(deg);
        elbowMotor.setTargetPosition(elbowPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(power);
    }
    public void elbowJoystick(double joystickControl){
        int targetPos = (int) (elbowMotor.getCurrentPosition() + 100*joystickControl);
        goToTargetPosition(targetPos);
    }
    public void holdPosition(){
        //this only works because we have a worm gear and we don't want to waste uneccesary battery power
        elbowMotor.setPower(0);
        //goToTargetPosition(elbowMotor.getCurrentPosition());
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



    public class elbowControl implements Action {
        private final int target;
        private final double speed;

        elbowControl(int targetPos, double speed) {
            this.target = targetPos;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goTo(target, speed);
            if (target-30 < getElbowTicks() && getElbowTicks() < target+30) {
                elbowMotor.setPower(0);

                //holdPos();
                return false;
            } else {
                return true;
            }
        }


    }
    public Action elbowControl(int targetPos, double speed) {
        return new elbowControl(targetPos, speed);
    }



}
