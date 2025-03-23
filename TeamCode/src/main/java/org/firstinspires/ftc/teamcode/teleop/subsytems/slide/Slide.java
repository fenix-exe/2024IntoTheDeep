package org.firstinspires.ftc.teamcode.teleop.subsytems.slide;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.floor;

import org.firstinspires.ftc.teamcode.commonCode.CommonSlide;

public class Slide extends CommonSlide {
    public DcMotorEx leftSlideMotor;
    public DcMotorEx rightSlideMotor;
    public RevTouchSensor homingSwitch;
    public Slide(DcMotorEx leftSlideMotor,DcMotorEx rightSlideMotor, RevTouchSensor homingSwitch){
        super(leftSlideMotor,rightSlideMotor,homingSwitch);
        this.rightSlideMotor = rightSlideMotor;
    }
    public void joystickControl(double slideMovement) {
        int targetPos = (int) (rightSlideMotor.getCurrentPosition() + 200 * slideMovement);
        setSlideExtensionLengthInTicks(targetPos);
    }

}
