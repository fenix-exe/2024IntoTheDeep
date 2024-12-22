package org.firstinspires.ftc.teamcode.subsytems.wrist.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class WristTest extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Wrist wrist;
    double pitchPosition;
    double rollPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        rollServo = hardwareMap.get(Servo.class, "roll");
        wrist = new Wrist(pitchServo, rollServo);
        wrist.presetPositionPitch(0);
        wrist.presetPositionRoll(0);
        pitchPosition = 0;
        rollPosition = 0;

        waitForStart();

        while (opModeIsActive()){
            pitchPosition = pitchServo.getPosition();
            rollPosition = rollServo.getPosition();
            if (gamepad1.dpad_down) {
                wrist.manualControlPitch(-0.001, pitchPosition);
            }
            if (gamepad1.dpad_up) {
                wrist.manualControlPitch(0.001, pitchPosition);
            }
            if (gamepad1.dpad_left) {
                wrist.manualControlRoll(-0.001, rollPosition);
            }
            if (gamepad1.dpad_right) {
                wrist.manualControlRoll(0.001, rollPosition);
            }
            if (gamepad1.a) {
                wrist.presetPositionPitch(0);
                wrist.presetPositionRoll(0);
            }
        }
    }
}
