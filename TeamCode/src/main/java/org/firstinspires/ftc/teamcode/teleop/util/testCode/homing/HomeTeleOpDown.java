package org.firstinspires.ftc.teamcode.teleop.util.testCode.homing;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commonCode.Homing;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

@TeleOp
public class HomeTeleOpDown extends LinearOpMode {
    Elbow elbow;
    Slide slide;
    LinearActuator linearActuator;
    Servo pitch;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    DcMotorEx pivot;
    DcMotorEx linearActuatorMotor;
    RevTouchSensor elbowSwitch;
    RevTouchSensor slideSwitch;
    RevTouchSensor actuatorSwitch;
    Homing homing;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeArmAndHome();

        waitForStart();
        if (opModeIsActive()){
            homing.homeDown();
        }
    }


    private void initializeArmAndHome(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        elbowSwitch = hardwareMap.get(RevTouchSensor.class, "elbow switch");
        slideSwitch = hardwareMap.get(RevTouchSensor.class, "slide switch");
        actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");


        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = new Slide(leftSlide,rightSlide, slideSwitch);
        elbow = new Elbow(pivot, elbowSwitch,90);
        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

        homing = new Homing(leftSlide,rightSlide,pivot,linearActuatorMotor,this, telemetry, slideSwitch,actuatorSwitch,elbowSwitch);
        pitch = hardwareMap.get(Servo.class, "pitch");

        pitch.setPosition(0.5);
        while (opModeInInit()){
            telemetry.addData("homing switch", slide.isHomingSwitchPressed());
            telemetry.update();
        }

    }
    private void homeDown(){
        //homing the slide
        while (!slide.isHomingSwitchPressed() && !isStopRequested()){
            slide.setSlidePower(-600);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Homing the elbow
        while (!elbow.isLimitSwitchPressed() && !isStopRequested()){
            elbow.setElbowPower(-0.2);
        }
        while (elbow.isLimitSwitchPressed() && !isStopRequested()){
            elbow.setElbowPower(-0.4);
        }
        while (!elbow.isLimitSwitchPressed() && !isStopRequested()){
            elbow.setElbowPower(0.4);
        }
        elbow.setElbowPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot.setTargetPosition(-100);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(1);

        while((Math.abs(pivot.getCurrentPosition() - pivot.getTargetPosition()) > 12)){

        }

        pivot.setPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !isStopRequested()){
            telemetry.addLine("ELBOW IS HOMED");
            telemetry.update();
            linearActuator.setLinearActuatorPower(-0.5);
        }
        linearActuator.setLinearActuatorPower(0);

        linearActuator.resetEncoders();



    }
    private void homeUp(){
        //homing the slide
        while (!slide.isHomingSwitchPressed() && !isStopRequested()){
            slide.setSlidePower(-0.2);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Homing the elbow
        while (!elbow.isLimitSwitchPressed() && !isStopRequested()){
            elbow.setElbowPower(0.2);
        }
        elbow.setElbowPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !isStopRequested()){
            linearActuator.setLinearActuatorPower(-0.5);
        }
        linearActuator.setLinearActuatorPower(0);

        linearActuator.resetEncoders();



    }


}


