package org.firstinspires.ftc.teamcode.subsytems.wrist.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class WristTest extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Wrist wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        rollServo = hardwareMap.get(Servo.class, "roll");
        wrist = new Wrist(pitchServo, rollServo);
        wrist.presetPositionPitch(0);
        wrist.presetPositionRoll(0);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_down) {
                wrist.manualControlPitch(-1);
            }
            if (gamepad1.dpad_up) {
                wrist.manualControlPitch(1);
            }
            if (gamepad1.dpad_left) {
                wrist.manualControlRoll(-1);
            }
            if (gamepad1.dpad_right) {
                wrist.manualControlRoll(1);
            }
            if (gamepad1.a) {
                wrist.presetPositionPitch(0);
                wrist.presetPositionRoll(0);
            }
        }
    }
}
