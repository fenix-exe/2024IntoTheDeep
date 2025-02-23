package org.firstinspires.ftc.teamcode.teleop.util.testCode.homing;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

@TeleOp
@Disabled
public class HomeTeleOpUp extends LinearOpMode {
    Elbow elbow;
    Slide slide;
    LinearActuator linearActuator;
    Servo pitch;
    DcMotorEx LeftSlideMotor;
    DcMotorEx RightSlideMotor;
    DcMotorEx pivot;
    DcMotorEx linearActuatorMotor;
    RevTouchSensor limitSwitch;
    RevTouchSensor homingSwitch;
    RevTouchSensor actuatorSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeArmAndHome();

        waitForStart();
        while (opModeIsActive()){
                homeUp();
                slide.setSlideExtensionLength(0);
                elbow.setTargetAngle(0);
                linearActuator.goToTargetPositionInches(0);
        }


    }


    private void initializeArmAndHome(){
        LeftSlideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");
        actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");


        LeftSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = new Slide(LeftSlideMotor,null, homingSwitch);
        elbow = new Elbow(pivot, limitSwitch,90);
        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

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
            slide.setSlidePower(-0.2);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        LeftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !isStopRequested()){
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

        LeftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


