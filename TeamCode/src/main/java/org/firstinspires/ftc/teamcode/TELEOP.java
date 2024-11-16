package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.driveCode;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

@TeleOp
public class TELEOP extends LinearOpMode {
    driveCode driverCode;
    activeIntake activeIntakeCode;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DriverControls controls;
    activeIntake intakeCode;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad gamepad1current;
    Gamepad gamepad2current;
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;
    DcMotorEx slide;
    DcMotorEx pivot;
    CRServo intake;
    ServoImplEx left;
    ServoImplEx right;
    differential diffCode;
    IMU imu;
    double MaxSlideExtensionInches;
    int initialTopHeight = 5000;
    int topHeight = 5000;
    int lastTopHeight = 5000;
    int topPivotPos = 2178;
    int slowDownPivotHeight = 1000;
    double pitchPos = 0;
    double pitchStep = 13.5;
    double rollStep = 13.5;
    double rollPos = 0;
    double speedMultiplication = 1;
    private enum driveType {FIELD, ROBOT}
    private enum speed {FAST, SLOW}
    private enum slidePos {UP, DOWN}
    private enum intakeDirection {FORWARD, BACKWARD}
    private enum intakePower {YES, NO}
    private enum pivotPos {DEPOSIT, PICKUP}
    driveType drive;
    speed speedMultiplier;
    slidePos slideUpOrDown;
    intakeDirection direction;
    intakePower power;
    pivotPos pivotStateMachine;

    @Override
    public void runOpMode() throws InterruptedException {

        //gamepad initializations
        gamepad1current = new Gamepad();
        gamepad2current = new Gamepad();

        gamepad1previous = new Gamepad();
        gamepad2previous = new Gamepad();

        //imu initializations
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        //motor intializations
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        intake = hardwareMap.get(CRServo.class, "intake");
        left = hardwareMap.get(ServoImplEx.class,"left");
        right = hardwareMap.get(ServoImplEx.class,"right");

        //reversing motor directions
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        //resetting encoders
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //zeroPowerBehavior
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gamepad copying
        gamepad1previous.copy(gamepad1);
        gamepad2previous.copy(gamepad2);

        driverCode = new driveCode(gamepad1, gamepad1previous, FL, FR, BL, BR, imu, telemetry);
        activeIntakeCode = new activeIntake(intake);
        slideCode = new slideCodeFunctions(slide);

        //pivot initialization
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, topPivotPos);

        //driveMap initialization
        controls = new DriverControls(gamepad1current, gamepad2current, gamepad1previous, gamepad2previous);

        //state machines initialization
        drive = driveType.ROBOT;
        speedMultiplier = speed.FAST;
        slideUpOrDown = slidePos.DOWN;
        direction = intakeDirection.FORWARD;
        power = intakePower.NO;
        pivotStateMachine = pivotPos.PICKUP;


        waitForStart();

        while (opModeIsActive()){
            gamepad1previous.copy(gamepad1current);
            gamepad2previous.copy(gamepad2current);

            gamepad1current.copy(gamepad1);
            gamepad2current.copy(gamepad2);

            controls = new DriverControls(gamepad1current, gamepad2current, gamepad1previous, gamepad2previous);

            if (controls.driveTypeSwitch()){
                if (drive == driveType.ROBOT){
                    drive = driveType.FIELD;
                } else {
                    drive = driveType.ROBOT;
                }
            }
            if (controls.slowMode() || pivot.getCurrentPosition() > slowDownPivotHeight){
                if (speedMultiplier == speed.FAST || pivot.getCurrentPosition() > slowDownPivotHeight){
                    speedMultiplier = speed.SLOW;
                } else {
                    speedMultiplier = speed.FAST;
                }
            }
//slide-pivot soft stop
            double theta = pivotCode.ticksToDegrees(pivot.getCurrentPosition());
            if (theta != 90) {
                MaxSlideExtensionInches = 36/(Math.cos(Math.toRadians(theta)));
            } else {
                MaxSlideExtensionInches = 10^44;
            }
            int MaxSlideExtensionEncoderTicks = slideCode.InchesToTicks(MaxSlideExtensionInches);
            if (MaxSlideExtensionEncoderTicks < initialTopHeight){
                topHeight = MaxSlideExtensionEncoderTicks;
            } else {
                topHeight = initialTopHeight;
            }
            /*if (slide.getCurrentPosition() > (topHeight - 100) && (lastTopHeight - 1 < topHeight) && (topHeight < lastTopHeight + 1)){
                slideCode.goTo(topHeight - 100);
            }*/
            lastTopHeight = topHeight;

            //Slide code

            if (controls.slidesFullyUp()){
                slideCode.goTo(topHeight);
            } else if (controls.slidesFullyDown()){
                slideCode.goTo(0);
            } else if ( Math.abs(controls.slideMovement()) > 0){
                slideCode.joystickControl(controls.slideMovement(), topHeight);
            } else if (slide.getCurrentPosition() > topHeight) {
                slideCode.goTo(topHeight);
            } else if (slide.getCurrentPosition() < 0){
                slideCode.goTo(0);
            } else {
                slideCode.holdPos();
            }

            if (controls.pivotParallel()){
                pivotCode.goTo(0);
            }
            if (controls.pivotPerp()){
                pivotCode.goTo(topPivotPos);
            }
            pivotCode.pivotJoystick(pivot.getCurrentPosition(), controls.pivotJoystick());
            if (controls.pivotJoystick() == 0){
                pivotCode.goTo(pivot.getCurrentPosition());
            }
            //State definitions
            if (controls.intakenewForward() > 0.5){
                direction = intakeDirection.FORWARD;
                power = intakePower.YES;
            } else if (controls.intakenewBackward() > 0.5){
                direction = intakeDirection.BACKWARD;
                power = intakePower.YES;
            } else {
                power = intakePower.NO;
            }
            if (pivot.getCurrentPosition() >= topPivotPos - 100){
                pivotStateMachine = pivotPos.DEPOSIT;
            } else{
                pivotStateMachine = pivotPos.PICKUP;
            }
            if (slide.getCurrentPosition() >= topHeight){
                slideUpOrDown = slidePos.UP;
            } else{
                slideUpOrDown = slidePos.DOWN;
            }

            /*if (controls.intakeDirection()){
                if (direction == intakeDirection.FORWARD){
                    direction = intakeDirection.BACKWARD;
                } else {
                    direction = intakeDirection.FORWARD;
                }
            }*/

            /*if (controls.intakePower()){
                if (power == intakePower.YES){
                    power = intakePower.NO;
                } else {
                    power = intakePower.YES;
                }
            }*/

            //switch statements for state machines
            switch (speedMultiplier){
                case SLOW:
                    speedMultiplication = 0.5;
                    break;
                default:
                    speedMultiplication = 1;
                    break;
            }

            switch (drive) {
                case FIELD:
                    driverCode.FieldCentricDrive(speedMultiplication, controls.resetIMU());
                    break;
                default:
                    driverCode.RobotCentric_Drive(speedMultiplication);
                    break;
            }

            switch (power){
                case NO:
                    activeIntakeCode.intakeOff();
                    break;
                case YES:
                    switch (direction){
                        case FORWARD:
                            activeIntakeCode.intakeForward();
                            break;
                        case BACKWARD:
                            activeIntakeCode.intakeBack();
                            break;
                        default:
                            activeIntakeCode.intakeOff();
                            break;
                    }
                    break;
                default:
                    activeIntakeCode.intakeOff();
                    break;
            }

            pitchPos += controls.degreeOfFreedomX() * pitchStep;
            rollPos += controls.degreeOfFreedomY() * rollStep;
            diffCode.setDifferentialPosition(pitchPos, rollPos);
            telemetry.addData("pitch", pitchPos);
            telemetry.addData("roll", rollPos);
            if (controls.resetWrist()) {
                diffCode.setDifferentialPosition(0,0);
            }
            telemetry.addData("max slide height", topHeight);
            telemetry.update();
        }


    }
}
