package org.firstinspires.ftc.teamcode.util.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

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
            diffCode.setDifferentialPosition(-90, 0);
            telemetry.addData("left", left.getPosition());
            telemetry.addData("g", gamepad2.left_stick_y);
            telemetry.addData("g2", gamepad2.right_stick_y);
            telemetry.addData("right", right.getPosition());
            telemetry.update();
        }
    }
}
