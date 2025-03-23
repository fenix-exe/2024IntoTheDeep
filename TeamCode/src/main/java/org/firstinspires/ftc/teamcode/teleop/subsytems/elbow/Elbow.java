package org.firstinspires.ftc.teamcode.teleop.subsytems.elbow;


import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Math.floor;

import org.firstinspires.ftc.teamcode.commonCode.CommonElbow;


public class Elbow extends CommonElbow {
    public DcMotorEx elbowMotor;
    public RevTouchSensor limitSwitch;
    public int topPosition;
    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, int topPositionInDegrees){
        super(elbow,limitSwitch,topPositionInDegrees);
        this.elbowMotor = elbow;
        this.topPosition = topPositionInDegrees;
    }
    public void elbowJoystick(double joystickControl){
        int targetPos = (int) (elbowMotor.getCurrentPosition() + 100*joystickControl);
        super.goToTargetPosition(targetPos);
    }





}
