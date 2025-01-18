package org.firstinspires.ftc.teamcode.teleop.subsytems.test;
//imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BackLeftDrivetrainMotor")
public class BackLeftDrivetrainMotor extends LinearOpMode {

    private DcMotor BL;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        float drive;
        double strafe;
        float yaw;
        double denominator;

        BL = hardwareMap.get(DcMotor.class, "BL");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                yaw = gamepad1.right_stick_x;
                drive = -gamepad1.left_stick_y;
                strafe = gamepad1.right_stick_x * 1.1;
                denominator = Math.max(1, Math.abs(drive + strafe + yaw));
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setPower((drive - strafe + yaw) / denominator);
            }
            telemetry.addLine("BL Motor Test");
            telemetry.update();
        }
    }
}
