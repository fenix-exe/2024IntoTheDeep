package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

public class Homing {
    private final DcMotorEx leftSlideMotor;
    private final DcMotorEx rightSlideMotor;
    private final Slide slide;
    private final DcMotorEx elbowMotor;
    private final DcMotorEx linearActuatorMotor;
    private final LinearActuator linearActuator;
    private final Elbow elbow;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    public Homing(DcMotorEx leftSlide, DcMotorEx rightSlide, DcMotorEx elbowMotor, DcMotorEx linearActuator, LinearOpMode opMode, Telemetry telemetry, RevTouchSensor slideHoming, RevTouchSensor linearActuatorHoming, RevTouchSensor elbowHoming){
        this.leftSlideMotor = leftSlide;
        this.rightSlideMotor = rightSlide;
        this.elbowMotor = elbowMotor;
        this.linearActuatorMotor = linearActuator;
        this.opMode = opMode;
        this.telemetry = telemetry;
        slide = new Slide(this.leftSlideMotor, this.rightSlideMotor, slideHoming);
        elbow = new Elbow(this.elbowMotor, elbowHoming, 100);
        this.linearActuator = new LinearActuator(linearActuatorMotor,linearActuatorHoming);
    }

    public String slideNullCheck() {
        return rightSlideMotor.getDeviceName();
    }

    public String elbowNullCheck() {
        return elbowMotor.getDeviceName();
    }

    public void homeDown(){
        //homing the slide
        while (!slide.isHomingSwitchPressed() && !opMode.isStopRequested()){
            slide.setSlidePower(-0.3);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        slide.resetEncoder();

        //Homing the elbow
        while (!elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            elbow.setElbowPower(-0.2);
        }
        while (elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            elbow.setElbowPower(-0.4);
        }
        while (!elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            elbow.setElbowPower(0.4);
        }
        elbow.setElbowPower(0);

        elbow.resetEncoder();

        elbow.setTargetAngle(elbow.ticksToDegrees(-100));

        while((Math.abs(elbow.getElbowAngle() - elbow.getElbowTargetAngle()) > elbow.ticksToDegrees(12))){

        }

        elbow.setElbowPower(0);

        elbow.resetEncoder();

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !opMode.isStopRequested()){
            telemetry.addLine("ELBOW IS HOMED");
            telemetry.update();
            linearActuator.setLinearActuatorPower(-0.5);
        }
        linearActuator.setLinearActuatorPower(0);

        linearActuator.resetEncoders();



    }
}
