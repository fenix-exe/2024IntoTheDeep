package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.common.BarkerIntake;

public class BarkerTest extends LinearOpMode {
    CRServo servo;
    BarkerIntake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "intake");
        intake = new BarkerIntake(servo);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.left_trigger > 0.1){
                intake.intake();
            } else if (gamepad1.right_trigger > 0.1){
                intake.outtake();
            } else {
                intake.stop();
            }
        }
    }
}
