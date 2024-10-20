package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

public class slideCodeFunctions {
    DcMotorEx slide;
    int liftPos;
    int maxHeight = 4000;
    int minHeight = 0;
    public slideCodeFunctions(DcMotorEx slide){
        this.slide=slide;
    }
    public void goTo(int targetPos){
        slide.setPower(1);
        slide.setTargetPosition(targetPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void joystickControl(double slideMovement){
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
}
