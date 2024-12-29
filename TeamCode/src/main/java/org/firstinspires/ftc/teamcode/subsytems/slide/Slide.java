package org.firstinspires.ftc.teamcode.subsytems.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Math.floor;


import androidx.annotation.NonNull;

public class Slide {
    public DcMotorEx slideMotor;
    double maxPhysicalExtensionInches;
    int maxPhysicalExtensionTicks;
    //pulleyCirc is the circumference of the pulley
    double PULLEYCIRC= 4.724757;
    //encoderRes is how many encoder ticks happen after 1 rotation of the motor
    double ENCODERRES = 384.5;
    //slideLength is the length of 1 stage of the slides
    //300 mm is the length of a misumi 330 slide, and 1 in = 25.4 mm
    double SLIDELENGTH = 300/25.4;
    //slideToElbow is the distance from the pivot point (center of axle) to the start of the slides
    double SLIDETOELBOW = 2.5;
    int currentTargetPos;
    public Slide(DcMotorEx slideMotor, double maxPhysicalExtensionInches){
        this.slideMotor =slideMotor;
        this.maxPhysicalExtensionInches = maxPhysicalExtensionInches;
        maxPhysicalExtensionTicks = inchesToTicks(maxPhysicalExtensionInches);
    }
    public void setSlideExtensionLength(double lengthInInches){
        int targetPosition = inchesToTicks(lengthInInches);
        setSlideExtensionLengthInTicks(targetPosition);
    }
    private void setSlideExtensionLengthInTicks(int ticks){
        slideMotor.setTargetPosition(ticks);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }

    public void joystickControl(double slideMovement){

        int targetPos = (int) (slideMotor.getCurrentPosition() + 200*slideMovement);
        setSlideExtensionLengthInTicks(targetPos);
    }
    public void holdPosition(){
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
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
        return ticksToInches(slideMotor.getCurrentPosition());
    }
    public void resetEncoder(){
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class slideControl implements Action {
        private final double targetPos;
        slideControl(double targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setSlideExtensionLength(targetPos);
            if (targetPos-0.5 < slideMotor.getCurrentPosition() && slideMotor.getCurrentPosition() < targetPos+0.5) {
                setSlideExtensionLength(targetPos);
                return false;
            }
            else {
                return true;
            }
        }
    }
    public Action slideControl(double targetPos){
        return new slideControl(targetPos);
    }
    
}
