package org.firstinspires.ftc.teamcode.subsytems.arm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class ElbowTest extends LinearOpMode {
    DcMotorEx elbowMotor;
    Elbow elbow;
    @Override
    public void runOpMode() throws InterruptedException {
        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow = new Elbow(elbowMotor, 2100);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                elbow.setTargetAngle(90);
            }
            if(gamepad1.b){
                elbow.setTargetAngle(0);
            }
            if(gamepad1.x){
                elbow.setTargetAngle(45);
            }
        }
    }
}
