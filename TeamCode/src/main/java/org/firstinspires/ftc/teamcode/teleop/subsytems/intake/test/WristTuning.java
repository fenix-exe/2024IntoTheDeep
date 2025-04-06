package org.firstinspires.ftc.teamcode.teleop.subsytems.intake.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

@Config
@TeleOp
@Disabled
public class WristTuning extends LinearOpMode {
    Servo pitchLeft;
    Servo pitchRight;
    Wrist wrist;
    MultipleTelemetry multipleTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        pitchLeft = hardwareMap.get(Servo.class, "pitchLeft");
        pitchRight = hardwareMap.get(Servo.class, "pitchRight");
        wrist = new Wrist(pitchLeft);
        multipleTelemetry = new MultipleTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                wrist.manualControlPitch(0.005);
            }
            if (gamepad1.dpad_down){
                wrist.manualControlPitch(-0.005);
            }
            if (gamepad1.a){
                wrist.presetPositionPitch(0.5);
            }
            multipleTelemetry.addData("Left Servo Pos", pitchLeft.getPosition());
            multipleTelemetry.addData("Right Servo Pos", pitchRight.getPosition());
            multipleTelemetry.update();
        }
    }
}
