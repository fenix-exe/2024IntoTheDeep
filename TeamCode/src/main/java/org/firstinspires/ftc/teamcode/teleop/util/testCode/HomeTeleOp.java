package org.firstinspires.ftc.teamcode.teleop.util.testCode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

@TeleOp
public class HomeTeleOp extends LinearOpMode {
    Elbow elbow;
    Slide slide;
    Servo pitch;
    DcMotorEx slideMotor;
    DcMotorEx pivot;
    RevTouchSensor limitSwitch;
    RevTouchSensor homingSwitch;

    private void initializeArmAndHome(){
        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");

        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = new Slide(slideMotor, homingSwitch);
        elbow = new Elbow(pivot, limitSwitch,90);

        pitch = hardwareMap.get(Servo.class, "pitch");

        pitch.setPosition(0.5);
        while (opModeInInit()){
            telemetry.addData("homing switch", slide.getHomingSwitchState());
            telemetry.update();
        }
        waitForStart();

        home();

        slideMotor.setTargetPosition(0);
        pivot.setTargetPosition(0);
    }
    private void home(){
        //homing the slide
        while (!slide.getHomingSwitchState() && !isStopRequested()){
            slide.setSlidePower(-0.2);
            telemetry.addData("slide switch state", slide.getHomingSwitchState());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Homing the elbow
        while (!elbow.getLimitSwitchState() && !isStopRequested()){
            elbow.setElbowPower(0.2);
        }
        elbow.setElbowPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeArmAndHome();
    }
}

