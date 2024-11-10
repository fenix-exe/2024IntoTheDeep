package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class differentialTeleop extends LinearOpMode {
    double pitch;
    double roll;
    ServoImplEx left;
    ServoImplEx right;
    differential differentialCode;
    Gamepad gamepad1current;
    Gamepad gamepad1previous;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1current = new Gamepad();
        gamepad1previous = new Gamepad();

        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        differentialCode = new differential(left, right);
        double pitch;
        double roll;


        differentialCode.setDifferentialPosition(-90,90);
        roll = -90;
        pitch = 90;
        boolean dontmoveroll = false;
        gamepad1current.copy(gamepad1);
        waitForStart();

        while (opModeIsActive()){
            dontmoveroll = false;
            gamepad1previous.copy(gamepad1current);
            gamepad1current.copy(gamepad1);
            if (gamepad1.a && !gamepad1previous.a){
                telemetry.addLine("going to 0,0");
                pitch = 0;
                roll = 0;
            }
            if (gamepad1.b && !gamepad1previous.b){
                telemetry.addLine("going to -90,90");
                pitch = -90;
                roll = 90;

            }
            if (gamepad1.x && !gamepad1previous.x){
                telemetry.addLine("going to 90,90");
                roll = -90;
                pitch = 90;
             }
            if (gamepad1.y && !gamepad1previous.y){
                telemetry.addLine("going to -45,90");
                roll = 90;
                pitch = -45;
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