package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp
public class differentialTeleop extends LinearOpMode {
    ServoImplEx left;
    ServoImplEx right;
    CRServo intake;
    differential differentialCode;
    @Override
    public void runOpMode() throws InterruptedException {


        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
        intake = hardwareMap.get(CRServo.class, "intake");

        differentialCode = new differential(left, right);
        double pitch;
        double roll;

        pitch = 0;
        roll = 0;


        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                //zero
                telemetry.addLine("going to 0,0");
                pitch = 0;
                roll = 0;
            }
            if (gamepad1.b){

                telemetry.addLine("going to -90,90");
                //outtake
                pitch = -90;
                roll = -90;

            }
            if (gamepad1.x){
                //ascentpark fold in
                telemetry.addLine("going to 90,90");
                pitch = 90;
                roll = -90;

            }
            if (gamepad1.y){
                telemetry.addLine("going to -45,90");
                //intake
                pitch = 50;
                roll = 90;


            }

            if (gamepad1.left_bumper) {
                intake.setPower(1);
            }
            else if (gamepad1.right_bumper) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            differentialCode.setDifferentialPosition(pitch, roll);


            telemetry.addData("Left pos", left.getPosition());
            telemetry.addData("Right pos", right.getPosition());
            telemetry.update();
        }
    }
}