package org.firstinspires.ftc.teamcode.commonCode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

public class Homing {
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private Slide slide;
    private DcMotorEx elbowMotor;
    private DcMotorEx linearActuatorMotor;
    private LinearActuator linearActuator;
    private Elbow elbow;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    public Homing(DcMotorEx leftSlide, DcMotorEx rightSlide, DcMotorEx elbow, DcMotorEx linearActuator, LinearOpMode opMode, Telemetry telemetry, RevTouchSensor slideHoming, RevTouchSensor linearActuatorHoming, RevTouchSensor elbowHoming){
        this.leftSlideMotor = leftSlide;
        this.rightSlideMotor = rightSlide;
        this.elbowMotor = elbow;
        this.linearActuatorMotor = linearActuator;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.slide = new Slide(leftSlideMotor, rightSlideMotor, slideHoming);
        this.elbow = new Elbow(elbowMotor, elbowHoming, 100);
        this.linearActuator = new LinearActuator(linearActuatorMotor,linearActuatorHoming);

    }
    public void homeDown(){
        //homing the slide
        while (!slide.isHomingSwitchPressed() && !opMode.isStopRequested()){
            slide.setSlidePower(-600);
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
