package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

@TeleOp
public class slideTeleop extends LinearOpMode {

    DcMotorEx slides;
    DcMotorEx elbow;
    int topHeight = 5000;
    public enum slidePos{MOVING_TO_POSITION, JOYSTICK_CONTROL}
    slidePos slideUpOrDown;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = hardwareMap.get(DcMotorEx.class, "slide");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow = hardwareMap.get(DcMotorEx.class, "pivot");
        slideUpOrDown = slidePos.MOVING_TO_POSITION;
        slideCodeFunctions slideCode = new slideCodeFunctions(slides);
        DriverControls controls = new DriverControls(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controls.update();
            //Slide code
            if (controls.slidesFullyUp()) {
                slideCode.goTo(topHeight);
                telemetry.addLine("Extending slides to the max");
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (controls.slidesFullyDown()) {
                slideCode.goTo(0);
                telemetry.addLine("Retracting slides fully");
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (Math.abs(-gamepad2.left_stick_y) > 0.5) {
                slideCode.joystickControl(-gamepad2.left_stick_y, topHeight);
                telemetry.addLine("Moving based off joystick");
                telemetry.addData("Joystick control", -gamepad2.left_stick_y);
                telemetry.addData("Current slide pos", slides.getCurrentPosition());
                telemetry.addData("Target slide pos", slides.getTargetPosition());
                slideUpOrDown = slidePos.JOYSTICK_CONTROL;
            } else if (slides.getCurrentPosition() > topHeight) {
                slideCode.goTo(topHeight);
                telemetry.addLine("Going to top height");
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (slides.getCurrentPosition() < 0) {
                slideCode.goTo(0);
                telemetry.addLine("Going to 0");
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else {
                telemetry.addLine("Holding pos");
                slideCode.holdPos();
            }
            telemetry.addData("topHeight", topHeight);
            telemetry.addData("inches", slideCode.ticksToInches(slides.getCurrentPosition()));
            telemetry.addData("ticks", slides.getCurrentPosition());
            telemetry.update();
        }

    }
}
