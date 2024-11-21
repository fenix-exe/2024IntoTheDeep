package org.firstinspires.ftc.teamcode.subsytems.arm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.floor;


import androidx.annotation.NonNull;

public class Slide {
    DcMotorEx slideMotor;
    double maxPhysicalExtensionInches;
    int maxPhysicalExtensionTicks;
    int minHeight = 0;
    //pulleyCirc is the circumference of the pulley
    double PULLEYCIRC= 4.724757;
    //encoderRes is how many encoder ticks happen after 1 rotation of the motor
    double ENCODERRES = 384.5;
    //slideLength is the length of 1 stage of the slides
    //300 mm is the length of a misumi 330 slide, and 1 in = 25.4 mm
    double SLIDELENGTH = 300/25.4;
    //slideToElbow is the distance from the pivot point (center of axle) to the start of the slides
    double SLIDETOELBOW = 2.5;
    static int STEP_SIZE_FOR_SLIDE = 400;
    public Slide(DcMotorEx slideMotor, double maxPhysicalExtensionInches){
        this.slideMotor =slideMotor;
        this.maxPhysicalExtensionInches = maxPhysicalExtensionInches;
        maxPhysicalExtensionTicks = inchesToTicks(maxPhysicalExtensionInches);
    }
    public void setSlideExtensionLength(double lengthInInches){
        int targetPosition = inchesToTicks(lengthInInches);
        setSlideExtensionLengthInTicks(targetPosition);
    }
    public void setSlideExtensionLengthInTicks(int ticks){
        if (ticks < 0 || ticks > maxPhysicalExtensionTicks){
            return;
        }

        if(slideMotor.getTargetPosition() == ticks) {
            return; //slide is already moving to our target
        }

        slideMotor.setTargetPosition(ticks);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void joystickControl(double slideMovement , int maxHeight){
        int newSlidePosition = slideMotor.getCurrentPosition() + (int) (STEP_SIZE_FOR_SLIDE * slideMovement);

        if(newSlidePosition > maxHeight){
            newSlidePosition = maxHeight;
        }
        if(newSlidePosition < minHeight) {
            newSlidePosition = minHeight;
        }
        setSlideExtensionLengthInTicks(newSlidePosition);

        /*double power;
        if (slideMotor.getCurrentPosition() > maxHeight - 100 && slideMovement > 0){
            power = 0;
        } else if (slideMotor.getCurrentPosition() < 100 && slideMovement < 0){
            power = 0;
        } else {
            power = slideMovement;
        }
        slideMotor.setPower(power);*/
    }
    public void holdPosition(){
        //setSlideExtensionLengthInTicks(slideMotor.getCurrentPosition());
        slideMotor.setTargetPosition(slideMotor.getTargetPosition());
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double ticksToInchesPivotPoint(int ticks){
        //pulleyCirc/encoderRes * ticks + singleStageSlideLength + slideToPivot
        return PULLEYCIRC/ENCODERRES * ticks + SLIDELENGTH + SLIDETOELBOW;
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
    public int getSlidePosition(){
        return slideMotor.getCurrentPosition();
    }
    public double getSlideLengthInInchesFromPivot(){
        return ticksToInchesPivotPoint(slideMotor.getCurrentPosition());
    }
    public double getSlideExtensionInInches(){
        return ticksToInches(slideMotor.getCurrentPosition());
    }
    public boolean isBusy(){
        return slideMotor.isBusy();
    }



    public class slideControl implements Action {
        private final int targetPos;
        slideControl(int targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setSlideExtensionLength(targetPos);
            holdPosition();
            return true;

        }

    }
    public Action slideControl(int targetPos){
        return new slideControl(targetPos);
    }

}
