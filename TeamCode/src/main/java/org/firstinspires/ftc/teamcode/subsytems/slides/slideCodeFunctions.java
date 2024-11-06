package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.floor;


import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

public class slideCodeFunctions {
    DcMotorEx slide;
    int liftPos;
    int minHeight = 0;
    //pulleyCirc is the circumference of the pulley
    double PULLEYCIRC= 4.724757;
    //encoderRes is how many encoder ticks happen after 1 rotation of the motor
    double ENCODERRES = 751.8;
    //slideLength is the length of 1 stage of the slides
    double SLIDELENGTH = 300/25.4;
    //slideToElbow is the distance from the pivot point (center of axle) to the start of the slides
    double SLIDETOELBOW = 1;
    public slideCodeFunctions(DcMotorEx slide){
        this.slide=slide;
    }
    public void goTo(int targetPos){
        slide.setPower(1);
        slide.setTargetPosition(targetPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void joystickControl(double slideMovement , int maxHeight){
        liftPos += 400 * slideMovement;

        if(liftPos > maxHeight){
            liftPos = maxHeight;
        }
        if(liftPos < minHeight){
            liftPos = minHeight;
        }
        goTo(liftPos);
    }
    public void holdPos(){
        liftPos = slide.getCurrentPosition();
        goTo(liftPos);
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
}
