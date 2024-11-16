package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;

@TeleOp
public class differentialTeleop extends LinearOpMode {
    double pitch;
    double roll;
    ServoImplEx left;
    ServoImplEx right;
    CRServo intake;
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
        intake = hardwareMap.get(CRServo.class, "intake");
        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
        differential differential = new differential(left, right);
        activeIntake aIntake = new activeIntake(intake);
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
                differentialCode.setDifferentialPosition(pitch,roll);
                timer.reset();
                dontmoveroll = true;
                roll = 0;
            }
            if (gamepad1.b && !gamepad1previous.b){
                telemetry.addLine("going to -90,90");
                pitch = -90;

                roll = -90;

            }
            if (gamepad1.x && !gamepad1previous.x){
                telemetry.addLine("going to 90,90");
                roll = -90;
                differentialCode.setDifferentialPosition(pitch,roll);
                timer.reset();
                dontmoveroll = true;
                pitch = 90;
             }
            if (gamepad1.y && !gamepad1previous.y){
                telemetry.addLine("going to -45,90");
                roll = 90;
                differentialCode.setDifferentialPosition(pitch,roll);
                timer.reset();
                dontmoveroll = true;
                pitch = -45;
            }

           /* if (gamepad1.left_trigger > 0.5) {
                aIntake.intakeForward();
            } else if (gamepad1.right_trigger > 0.5) {
                aIntake.intakeBack();
            } else {

            }*/
            intake.setPower(1);


            differential.setDifferentialPosition(pitch, roll);

            telemetry.addData("Left pos", left.getPosition());
            telemetry.addData("Right pos", right.getPosition());
            telemetry.update();
        }
    }
}