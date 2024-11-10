package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class differentialTeleop extends LinearOpMode {
    double pitch;
    double roll;
    Servo left;
    Servo right;
    differential differentialCode;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        differentialCode = new differential(left, right);
        double pitch;
        double roll;

        differentialCode.setDifferentialPosition(0,0);
        roll = 0;
        pitch = 0;
        boolean dontmoveroll = false;


        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                telemetry.addLine("going to 0,0");
                pitch = 0;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(300);
                roll = 0;
                differentialCode.setDifferentialPosition(pitch,roll);
            }
            if (gamepad1.b){
                telemetry.addLine("going to -90,90");
                pitch = -90;
                differentialCode.setDifferentialPosition(pitch, roll);
                timer.reset();
                dontmoveroll = true;
                roll = 90;

            }
            if (gamepad1.x){
                telemetry.addLine("going to 90,90");
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(300);
                pitch = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
            }
            if (gamepad1.y){
                telemetry.addLine("going to -45,90");
                pitch = -45;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(300);
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
            }

            if (timer.milliseconds() > 300 && !dontmoveroll){
                telemetry.addLine("Movingroll");
                differentialCode.setDifferentialPosition(pitch, roll);
            }


            telemetry.addData("Left pos", left.getPosition());
            telemetry.addData("Right pos", right.getPosition());
            telemetry.update();
        }
    }
}
