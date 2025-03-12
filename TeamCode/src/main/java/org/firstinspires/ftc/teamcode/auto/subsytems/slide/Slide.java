package org.firstinspires.ftc.teamcode.auto.subsytems.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Math.floor;


import androidx.annotation.NonNull;

public class Slide {
    public DcMotorEx leftSlideMotor;
    public DcMotorEx rightSlideMotor;
    public RevTouchSensor homingSwitch;
    //pulleyCirc is the circumference of the pulley
    double PULLEYCIRC= 4.724757;
    //encoderRes is how many encoder ticks happen after 1 rotation of the motor
    double ENCODERRES = 145.1; // old value is 384.5;
    //slideLength is the length of 1 stage of the slides
    //300 mm is the length of a misumi 330 slide, and 1 in = 25.4 mm
    double SLIDELENGTH = 300/25.4;
    //slideToElbow is the distance from the pivot point (center of axle) to the start of the slides
    public double SLIDE_VEL = 3000;
    double SLIDETOELBOW = 2.5;

    public Slide(DcMotorEx leftSlideMotor,DcMotorEx rightSlideMotor, RevTouchSensor homingSwitch){
        this.leftSlideMotor = leftSlideMotor;
        this.rightSlideMotor = rightSlideMotor;
        this.homingSwitch = homingSwitch;
    }
    public void setSlideExtensionLength(double lengthInInches){
        int targetPosition = inchesToTicks(lengthInInches);
        setSlideExtensionLengthInTicks(targetPosition);
    }
    private void setSlideExtensionLengthInTicks(int ticks){
        rightSlideMotor.setTargetPosition(ticks);
        leftSlideMotor.setTargetPosition(ticks);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setVelocity(SLIDE_VEL);
        leftSlideMotor.setVelocity(SLIDE_VEL);
    }

    public void joystickControl(double slideMovement){
        int targetPos = (int) (rightSlideMotor.getCurrentPosition() + 400*slideMovement);
        setSlideExtensionLengthInTicks(targetPos);
    }
    public void holdPosition(){
        rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition());
        leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition());
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setPower(1);
        leftSlideMotor.setPower(1);
    }
    public int inchesToTicksPivotPoint(double inches){
        //encoderRes * (inches - slideLength - slideToElbow)/pulleyCirc
        return (int) floor(ENCODERRES*(inches-SLIDELENGTH - SLIDETOELBOW)/PULLEYCIRC);
    }

    public double ticksToInches(int ticks){
        return PULLEYCIRC/ENCODERRES*ticks;
    }
    public int inchesToTicks(double inches){
        return (int) floor(ENCODERRES/PULLEYCIRC*inches);
    }
    public double getSlideExtensionInInches(){
        return ticksToInches(rightSlideMotor.getCurrentPosition());
    }
    public void resetEncoder(){
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setSlidePower(double power){
        rightSlideMotor.setPower(power);
        leftSlideMotor.setPower(power);
    }
    public boolean isHomingSwitchPressed(){
        return homingSwitch.isPressed();
    }
    public void setLeftSlideMotorPowerToRightSlideMotorPower(){
        leftSlideMotor.setPower(rightSlideMotor.getPower());
    }


    public class slideControl implements Action {
        private final double targetPos;
        slideControl(double targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setSlideExtensionLength(targetPos);
            if (homingSwitch.isPressed()){
                leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setSlideExtensionLength(targetPos);
            }
            return !(targetPos - 0.5 < getSlideExtensionInInches()) || !(getSlideExtensionInInches() < targetPos + 0.5);
        }
    }
    public Action slideControl(double targetPos){
        return new slideControl(targetPos);
    }


}
