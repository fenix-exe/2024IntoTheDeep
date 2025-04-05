package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
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
    private final double LOW_ELBOW_POS = -12;
    private final double ELBOW_TOLERANCE = 0.5;
    private final double SLIDE_EXTENSION_LENGTH = 10;
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

    public void moveElbowUpAndHomeDown(){

        //homing the slide
        while (!slide.isHomingSwitchPressed() && !opMode.isStopRequested()){
            slide.setSlidePower(-0.3);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        slide.resetEncoder();

        elbow.resetEncoder();
        elbow.setTargetAngle(45);
        while ((Math.abs(elbow.getElbowAngle() - elbow.getElbowTargetAngle()) > RobotConstants.ELBOW_TOLERANCE)) {

        }
        elbow.setElbowPower(0);
        elbow.resetEncoder();
        //Homing the elbow
        while (!elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            telemetry.addLine("ELBOW MOVING DOWN");
            elbow.setElbowPower(-0.2);
        }
        slide.setSlideExtensionLength(SLIDE_EXTENSION_LENGTH);
        while (elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            elbow.setElbowPower(-0.4);
        }
        while (!elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            telemetry.addLine("ELBOW MOVING UP");
            telemetry.update();
            elbow.setElbowPower(0.1);
        }
        elbow.setElbowPower(0);

        elbow.resetEncoder();

        elbow.setTargetAngle(LOW_ELBOW_POS);

        while((Math.abs(elbow.getElbowAngle() - elbow.getElbowTargetAngle()) > ELBOW_TOLERANCE)){

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

        slide.setSlideExtensionLength(0);

        while ((Math.abs(slide.getSlideExtensionInInches() - slide.getSlideTargetPositionInInches())) > RobotConstants.SLIDE_TOLERANCE){

        }

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


        elbow.resetEncoder();
        //Homing the elbow
        while (!elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            telemetry.addLine("ELBOW MOVING DOWN");
            elbow.setElbowPower(-0.2);
        }
        slide.setSlideExtensionLength(SLIDE_EXTENSION_LENGTH);
        while (elbow.isLimitSwitchPressed() && !opMode.isStopRequested()){
            elbow.setElbowPower(-0.4);
        }
        while ((!elbow.isLimitSwitchPressed()) && !opMode.isStopRequested()){
            elbow.setElbowPower(0.4);
        }
        elbow.setElbowPower(0);

        elbow.resetEncoder();

        elbow.setTargetAngle(LOW_ELBOW_POS);

        while((Math.abs(elbow.getElbowAngle() - elbow.getElbowTargetAngle()) > ELBOW_TOLERANCE)){

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

        slide.setSlideExtensionLength(0);

        while ((Math.abs(slide.getSlideExtensionInInches() - slide.getSlideTargetPositionInInches())) > RobotConstants.SLIDE_TOLERANCE){

        }

    }
}
