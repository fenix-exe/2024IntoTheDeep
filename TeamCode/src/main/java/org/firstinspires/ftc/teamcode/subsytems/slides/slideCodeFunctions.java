package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

public class slideCodeFunctions {
    DcMotorEx slide;
    int liftPos;
    DriverControls control;
    public slideCodeFunctions(DcMotorEx slide, DriverControls control){
        this.slide=slide;
        this.control=control;
    }
    public void goTo(int targetPos){
        slide.setPower(1);
        slide.setTargetPosition(targetPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void joystickControl(){
        liftPos += 400 * control.slideMovement();
        goTo(liftPos);
    }
    public void holdPos(){
        liftPos = slide.getTargetPosition();
        goTo(liftPos);
    }
}
