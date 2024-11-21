package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class HomeTeleOp extends LinearOpMode {
    DcMotorEx slide;
    DcMotorEx pivot;
    RevTouchSensor limitSwitch;
    ElapsedTime timer;

    private void initializeArmAndHome(){
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        home();

        slide.setTargetPosition(0);
        pivot.setTargetPosition(0);
    }
    private void home(){
        timer = new ElapsedTime();
        timer.reset();
        //Homing the elbow
        pivot.setTargetPosition(125);
        while (Math.abs(pivot.getCurrentPosition()) - 10 > pivot.getTargetPosition()){
            pivot.setPower(1);
        }
        while (!limitSwitch.isPressed() && !isStopRequested() && timer.milliseconds() < 5000){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeArmAndHome();
    }
}

