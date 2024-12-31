package org.firstinspires.ftc.teamcode.subsytems.slide.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "SlideCurrentExperiment (Blocks to Java)")
public class SlideCurrentExperiment extends LinearOpMode {
    MultipleTelemetry multiTelem;

    private DcMotorEx slide;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        multiTelem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide.setDirection(DcMotor.Direction.REVERSE);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (slide.getCurrentPosition() < 0 && -gamepad1.right_stick_y < 0) {
                    slide.setPower(0);
                } else if (slide.getCurrentPosition() > 3496 && -gamepad1.right_stick_y > 0) {
                    slide.setPower(0);
                } else {
                    slide.setPower(-gamepad1.right_stick_y);
                }
                multiTelem.addData("slide current", slide.getCurrent(CurrentUnit.MILLIAMPS));
                multiTelem.addData("slide encoder", slide.getCurrentPosition());
                // Put loop blocks here.
                multiTelem.update();
            }
        }
    }
}
