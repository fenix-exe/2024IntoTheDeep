package org.firstinspires.ftc.teamcode.teleop.subsytems.claw.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
@TeleOp
public class ClawTest extends LinearOpMode {
    Servo claw;
    Claw clawCode;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        clawCode = new Claw(claw);

        clawCode.openClaw();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                clawCode.openClaw();
            }
            if (gamepad1.b){
                clawCode.closeClaw();
            }
        }

    }
}
