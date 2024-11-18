package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.floor;


import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

import androidx.annotation.NonNull;

public class slideCodeFunctions {
    DcMotorEx slide;
    int liftPos;
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
    public slideCodeFunctions(DcMotorEx slide){
        this.slide=slide;
    }
    public void goTo(int targetPos){
        slide.setTargetPosition(targetPos);
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void joystickControl(double slideMovement , int maxHeight){
        liftPos = slide.getCurrentPosition() + (int) (50 * slideMovement);

        if(liftPos > maxHeight){
            liftPos = maxHeight;
        }
        if(liftPos < minHeight){
            liftPos = minHeight;
        }
        goTo(liftPos);
    }
    public void holdPos(){
        slide.setPower(0);
    }
    public double ticksToInches(int ticks){
        //pulleyCirc/encoderRes * ticks + singleStageSlideLength + slideToPivot
        return PULLEYCIRC/ENCODERRES * ticks + SLIDELENGTH + SLIDETOELBOW;
    }
    public int InchesToTicks(double inches){
        //encoderRes * (inches - slideLength - slideToElbow)/pulleyCirc
        return (int) floor(ENCODERRES*(inches-SLIDELENGTH - SLIDETOELBOW)/PULLEYCIRC);
    }
    public int getSlidePosition(){
        return slide.getCurrentPosition();
    }
    public double getSlideLengthInInches(){
        return ticksToInches(slide.getCurrentPosition());
    }

    public class slideControl implements Action {
        private final int targetPos;
        slideControl(int targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goTo(targetPos);
            if (targetPos-60 < getSlidePosition() && getSlidePosition() < targetPos+60) {
                goTo(targetPos);
                return false;
            }
            else {
                return true;
            }
        }
    }
    public Action slideControl(int targetPos){
        return new slideControl(targetPos);
    }

}
