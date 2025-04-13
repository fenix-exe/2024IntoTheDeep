package org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
@TeleOp
@Disabled
public class WristTest extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Wrist wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitchLeft");
        wrist = new Wrist(pitchServo);
        wrist.presetPositionPitch(0);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_down) {
                wrist.manualControlPitch(-0.02);
            }
            if (gamepad1.dpad_up) {
                wrist.manualControlPitch(0.02);
            }
            if (gamepad1.a) {
                wrist.presetPositionPitch(0.5);
            }
            sleep(100);
            telemetry.addData("Servo Pos", wrist.getPitchAngle());
            telemetry.addData("Offst",Wrist.SERVO_OFFSET);
            telemetry.update();
        }
    }
}
