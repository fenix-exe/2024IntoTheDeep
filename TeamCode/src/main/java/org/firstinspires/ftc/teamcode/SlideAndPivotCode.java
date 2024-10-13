package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp()
public class SlideAndPivotCode extends LinearOpMode {
    private DcMotorEx pivot;
    private DcMotorEx slide;
    double liftPos = 0;
    private void liftCode() {

        // Tune
        ((DcMotorEx) slide).setVelocity(2000);
        // Sets Speed
        if (gamepad1.a) {
            liftPos = 0;
            // Down Position
        } else if (gamepad1.b) {
            liftPos = 2100;
            // Up Position
        } else if (Math.abs(gamepad1.right_stick_y) > 0) {
            liftPos += 400 * -gamepad1.right_stick_y;
            // Gamepad Controls
        } else {
            liftPos = slide.getCurrentPosition();
            // Hold the Position
        }
        if (liftPos > 2100) {
            liftPos = 2100;
        }
        if (liftPos <= 1) {
            liftPos = 0;
        }
        // Gives a bit of safety without limiting us by a significant amount
        /*if (liftPos > SoftStopDistance() - 10) {
            liftPos = SoftStopDistance() - 10;
        }*/
        telemetry.addData("softStop", SoftStopDistance());
        telemetry.addData("liftPos", liftPos);
        // Sets Limits and Soft Stop
        slide.setTargetPosition((int) liftPos);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Makes the Slide Move
    }
    private double SoftStopDistance() {
        double MaxSlideExtensionEncoderTicks;
        double pivotAngleDegrees;
        double MaxSlideExtensionInches;

        pivotAngleDegrees = pivot.getCurrentPosition() / 29.33944;
        // Converts encoder ticks into values that can be used
        if (Math.cos(pivotAngleDegrees / 180 * Math.PI) == 0) {
            MaxSlideExtensionInches = 1e+44;
            // Creates a limit so high that it wont get reached
        } else {
            MaxSlideExtensionInches = 36 / Math.abs(Math.cos(pivotAngleDegrees / 180 * Math.PI));
            // gets the stop value in inches
        }
        MaxSlideExtensionEncoderTicks = 66.666666 * MaxSlideExtensionInches;
        // converts the stop value from inches to encoder ticks
        return MaxSlideExtensionEncoderTicks;
    }
    @Override
    public void runOpMode() {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide = hardwareMap.get(DcMotorEx.class, "slide");

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Resetting our Encoders
        pivot.setDirection(DcMotorEx.Direction.REVERSE);
        // Makes the pivot go in the correct direction
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                liftCode();
                telemetry.update();
            }
        }
    }
}
