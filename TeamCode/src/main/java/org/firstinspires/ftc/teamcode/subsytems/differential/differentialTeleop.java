package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class differentialTeleop extends LinearOpMode {
    double pitch;
    double roll;
    Servo left;
    Servo right;
    differential differentialCode;
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

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                telemetry.addLine("going to 0,0");
                pitch = 0;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(500);
                roll = 0;
                differentialCode.setDifferentialPosition(pitch,roll);
            }
            if (gamepad1.b){
                telemetry.addLine("going to -90,90");
                pitch = -90;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(500);
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);

            }
            if (gamepad1.x){
                telemetry.addLine("going to 90,90");
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(500);
                pitch = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
            }
            if (gamepad1.y){
                telemetry.addLine("going to -45,90");
                pitch = -45;
                differentialCode.setDifferentialPosition(pitch,roll);
                sleep(500);
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
            }
            telemetry.addData("Left pos", left.getPosition());
            telemetry.addData("Right pos", right.getPosition());
            telemetry.update();
        }
    }
}
