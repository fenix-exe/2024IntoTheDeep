package org.firstinspires.ftc.teamcode.teleop.subsytems.slide.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;
@Config
@TeleOp
public class SlideMaxVelocityFinder extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    RevTouchSensor touchSensor;
    public static double slideVel = 0.97;
    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        touchSensor = hardwareMap.get(RevTouchSensor.class, "homing switch");
        Slide slide = new Slide(leftSlide,rightSlide,touchSensor);
        waitForStart();

        while (opModeIsActive()){
            slide.joystickControl(-gamepad1.left_stick_y);
            telemetry.addData("Left Encoder", leftSlide.getCurrentPosition());
            telemetry.addData("Right Encoder", rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
