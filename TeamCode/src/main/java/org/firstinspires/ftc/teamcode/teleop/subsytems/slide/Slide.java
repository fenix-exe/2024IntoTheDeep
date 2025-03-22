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
    //pulleyCirc is the circumference of the pulley
    double PULLEYCIRC= 4.724757;
    //encoderRes is how many encoder ticks happen after 1 rotation of the motor
    double ENCODERRES = 145.1; // old value is 384.5;
    //slideLength is the length of 1 stage of the slides
    //300 mm is the length of a misumi 330 slide, and 1 in = 25.4 mm
    double SLIDELENGTH = 300/25.4;
    //slideToElbow is the distance from the pivot point (center of axle) to the start of the slides
    public double SLIDE_POWER = 0.97;
    double SLIDETOELBOW = 2.5;
    public Slide(DcMotorEx leftSlideMotor,DcMotorEx rightSlideMotor, RevTouchSensor homingSwitch){
        super(leftSlideMotor,rightSlideMotor,homingSwitch);
    }
    public void joystickControl(double slideMovement) {
        int targetPos = (int) (rightSlideMotor.getCurrentPosition() + 200 * slideMovement);
        setSlideExtensionLengthInTicks(targetPos);
    }

}
