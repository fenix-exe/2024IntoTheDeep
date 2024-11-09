package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.RobotArm;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.driveCode;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

@TeleOp
public class TELEOPV3 extends LinearOpMode {
    driveCode driverCode;
    activeIntake activeIntakeCode;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DriverControls controls;
    activeIntake intakeCode;
    RobotArm arm;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad gamepad1current;
    Gamepad gamepad2current;

    DcMotorEx slide;
    DcMotorEx pivot;
    CRServo intake;
    Servo left;
    Servo right;
    differential diffCode;
    IMU imu;
    RevColorSensorV3 activeIntakeSensor;
    TouchSensor limitSwitch;
    double MaxSlideExtensionInches;
    int PHYSICALMAXEXTENSION = 5000;
    int topHeight = 5000;
    //int lastTopHeight = 5000;
    int topPivotPos = 2178;
    int slowDownPivotHeight = 1000;
    double pitchPos = 0;
    double pitchStep = 20;
    double rollStep = 20;
    double rollPos = 0;
    double speedMultiplication = 1;
    private enum driveType {FIELD, ROBOT}
    private enum speed {FAST, SLOW}
    private enum slidePos {UP, DOWN, MOVING_TO_POSITION, JOYSTICK_CONTROL}
    private enum intakeDirection {FORWARD, BACKWARD}
    private enum intakePower {YES, NO}
    private enum pivotPos {DEPOSIT, PICKUP, MOVING_TO_POSITION, JOYSTICK_CONTROL}
    private enum targetBlockColor {RED, BLUE, YELLOW}
    driveType drive;
    speed speedMultiplier;
    slidePos slideUpOrDown;
    intakeDirection direction;
    intakePower power;
    pivotPos pivotStateMachine;
    targetBlockColor blockColor;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeIntake();
        initializeDifferential();

        //state machines initialization
        drive = driveType.ROBOT;
        speedMultiplier = speed.FAST;
        slideUpOrDown = slidePos.DOWN;
        direction = intakeDirection.FORWARD;
        power = intakePower.NO;
        pivotStateMachine = pivotPos.PICKUP;

        //uncomment this and line 241 for block selection
        /*while(!gamepad1.a && !gamepad1.b && !gamepad1.x){
            telemetry.addLine("Pressing A sets the target block color to red");
            telemetry.addLine("Pressing B sets the target block color to blue");
            telemetry.addLine("Pressing X sets the target block color to yellow");
            telemetry.update();
        }
        if (gamepad1.a){
            blockColor = targetBlockColor.RED;
        } else if (gamepad1.b){
            blockColor = targetBlockColor.BLUE;
        } else if (gamepad1.x){
            blockColor = targetBlockColor.YELLOW;
        }*/

        waitForStart();

        while (opModeIsActive()){
            controls.update();

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
            topHeight = arm.currentAllowedMaxExtensionLength();
            //Slide code
            if (controls.slidesFullyUp()){
                slideCode.goTo(topHeight);
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (controls.slidesFullyDown()){
                slideCode.goTo(0);
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (Math.abs(controls.slideMovement()) > 0 && slide.getTargetPosition() < PHYSICALMAXEXTENSION){
                slideCode.joystickControl(controls.slideMovement(), topHeight);
                slideUpOrDown = slidePos.JOYSTICK_CONTROL;
            } else if (slide.getCurrentPosition() > topHeight) {
                slideCode.goTo(topHeight);
            } else if (slide.getCurrentPosition() < 0){
                slideCode.goTo(0);
            } else if (slideUpOrDown != slidePos.MOVING_TO_POSITION) {
                slideCode.holdPos();
            }
//pivot code
            if (controls.pivotParallel()){
                if (arm.doesSlideNeedToRetract(0)){
                    slideCode.goTo(arm.getSlideMaxLength(0));
                }
                else {
                    pivotCode.goTo(0);
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                }
            } else if (controls.pivotPerp()){
                pivotCode.goTo(topPivotPos);
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
            } else if (controls.pivotJoystick() == 0 && pivotStateMachine != pivotPos.MOVING_TO_POSITION){
                pivot.setPower(0);
            } else if (controls.pivotJoystick() < 0){
                if (arm.doesSlideNeedToRetract(pivot.getCurrentPosition() - 50)){
                    slideCode.goTo(arm.getSlideMaxLength(pivot.getCurrentPosition() - 50));
                } else {
                    pivotCode.pivotJoystick(pivot.getCurrentPosition(), controls.pivotJoystick());
                    pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
                }
            } else if (controls.pivotJoystick() > 0) {
                pivotCode.pivotJoystick(pivot.getCurrentPosition(), controls.pivotJoystick());
                pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
            }
            if (controls.submersibleIntakeReady()){
                pivotCode.goTo(0);
                slideCode.goTo(796);
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                diffCode.setDifferentialPosition(0,90);
                pitchPos = 0;
                rollPos = 90;
            }
            if (controls.drivingPos()){
                pivotCode.goTo(pivotCode.degreesToTicks(45));
                slideCode.goTo(0);
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                //add differential code
                diffCode.setDifferentialPosition(90,90);
                pitchPos = 90;
                rollPos = 90;
            }
            if (controls.acsent1Park()){
                pivotCode.goTo(pivotCode.degreesToTicks(45));
                slideCode.goTo(796);
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                diffCode.setDifferentialPosition(-90,90);
                pitchPos = -90;
                rollPos = 90;
            }
            if (controls.depositReadyBack()){
                pivotCode.goTo(pivotCode.degreesToTicks(90));
                slideCode.goTo(slideCode.InchesToTicks(34));
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                diffCode.setDifferentialPosition(90,90);
                pitchPos = 90;
                rollPos = 90;
            }
            if(controls.depositReadyUp()){
                pivotCode.goTo(pivotCode.degreesToTicks(70));
                slideCode.goTo(slideCode.InchesToTicks(41));
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                diffCode.setDifferentialPosition(90,90);
                pitchPos = 90;
                rollPos = 90;
            }
            //State definitions
            if (controls.intakenewForward() > 0.5){
                direction = intakeDirection.FORWARD;
                power = intakePower.YES;
            } else if (controls.intakenewBackward() > 0.5/* || (blockColor == targetBlockColor.RED && activeIntakeSensor.red() >= 245 && activeIntakeSensor.green() <= 50) || (blockColor == targetBlockColor.BLUE && activeIntakeSensor.blue() >= 245) || (blockColor == targetBlockColor.RED && activeIntakeSensor.red() >= 245 && activeIntakeSensor.green() >= 245)*/){
                direction = intakeDirection.BACKWARD;
                power = intakePower.YES;
            } else {
                power = intakePower.NO;
            }
            /*if (pivot.getCurrentPosition() >= topPivotPos - 100){
                pivotStateMachine = pivotPos.DEPOSIT;
            } else{
                pivotStateMachine = pivotPos.PICKUP;
            }
            if (slide.getCurrentPosition() >= topHeight){
                slideUpOrDown = slidePos.UP;
            } else{
                slideUpOrDown = slidePos.DOWN;
            }*/

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

            if (rollPos > 180) {
                rollPos=180;
            } else if (rollPos < -180) {
                rollPos=-180;
            }
            if (pitchPos > 180) {
                pitchPos=180;
            } else if (pitchPos < -180) {
                pitchPos=-180;
            }


            diffCode.setDifferentialPosition(pitchPos, rollPos);
            telemetry.addData("pitch", pitchPos);
            telemetry.addData("roll", rollPos);
            telemetry.addData("max slide height", topHeight);
            telemetry.addData("elbow current draw", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("slide current draw", slide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }


    }

    private void initializeGamePads() {
        controls = new DriverControls(gamepad1, gamepad2);
    }

    private void initializeDriveTrain(){
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //imu initializations
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        driverCode = new driveCode(gamepad1, gamepad1previous, FL, FR, BL, BR, imu, telemetry);
    }
    private void initializeArmAndHome(){
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        //Homing the pivot
        /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);*/

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideCode = new slideCodeFunctions(slide);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, topPivotPos);
        arm = new RobotArm(slideCode, pivotCode, PHYSICALMAXEXTENSION, 35);
    }
    private void initializeIntake(){
        intake = hardwareMap.get(CRServo.class, "intake");
        //activeIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "activeIntakeSensor");
        activeIntakeCode = new activeIntake(intake);
    }
    private void initializeDifferential(){
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        diffCode = new differential(left, right);
    }
}
