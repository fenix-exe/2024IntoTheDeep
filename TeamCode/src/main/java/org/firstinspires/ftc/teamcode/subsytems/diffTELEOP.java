package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.differential;
@Disabled
public class diffTELEOP extends LinearOpMode {
    Servo left;
    Servo right;
    differential drive;;


    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");

        drive = new differential(left, right);
        left.setPosition(0.5);
        right.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()){
            drive.setDifferentialPosition(0, 0);

        }

    }
}