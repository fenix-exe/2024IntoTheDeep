package org.firstinspires.ftc.teamcode.subsytems.elbow.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;


@TeleOp
//@Disabled
public class ElbowTest extends LinearOpMode {
    DcMotorEx elbowMotor;
    Elbow elbow;
    @Override
    public void runOpMode() throws InterruptedException {
        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        RevTouchSensor limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow = new Elbow(elbowMotor, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0, 24.22), 2300);
        waitForStart();

        while (opModeIsActive()){
            elbowMotor.setPower(0);
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