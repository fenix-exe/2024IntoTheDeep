package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.common.PinchRollerIntake;
@Config
@TeleOp
public class PinchRollerTest extends LinearOpMode {
    CRServoImplEx servo;
    PinchRollerIntake intake;
    MultipleTelemetry multipleTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServoImplEx.class, "intake");
        intake = new PinchRollerIntake(servo);
        multipleTelemetry = new MultipleTelemetry();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.left_trigger > 0.1){
                intake.intake();
            } else if (gamepad1.right_trigger > 0.1){
                intake.outtake();
            } else {
                intake.stop();
            }
            multipleTelemetry.addData("Servo power", servo.getPower());
            multipleTelemetry.update();
        }
    }
}
