package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

@TeleOp
public class diffTELEOP extends LinearOpMode {
    Servo left;
    Servo right;
    differential diffCode;
    DriverControls controls;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(Servo.class,"left");
        right = hardwareMap.get(Servo.class, "right");
        diffCode = new differential(left, right);

        waitForStart();
        while (opModeIsActive()) {
            diffCode.setDifferentialPosition(gamepad2.left_stick_y*90, gamepad2.right_stick_y*90);
            telemetry.addData("left", left.getPosition());
            telemetry.addData("g", gamepad2.left_stick_y);
            telemetry.addData("g2", gamepad2.right_stick_y);
            telemetry.addData("right", right.getPosition());
            telemetry.update();
        }
    }
}
