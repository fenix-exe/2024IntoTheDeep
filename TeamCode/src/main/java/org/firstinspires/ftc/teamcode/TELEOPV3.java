package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.modules.RobotArm;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.driveCode;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

@TeleOp
@Disabled
public class TELEOPV3 extends LinearOpMode {
    driveCode driverCode;
    activeIntake activeIntakeCode;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DriverControls driverControls;
    activeIntake intakeCode;
    RobotArm arm;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad gamepad1current;
    Gamepad gamepad2current;

    DcMotorEx slide;
    DcMotorEx pivot;
    CRServo intake;
    ServoImplEx left;
    ServoImplEx right;
    differential diffCode;
    IMU imu;
    RevColorSensorV3 activeIntakeSensor;
    TouchSensor limitSwitch;
    ElapsedTime timer;
    double MaxSlideExtensionInches;
    int PHYSICALMAXEXTENSION = 2500;
    int maxAllowedExtension = 2500;
    //int lastTopHeight = 5000;
    int topPivotPos = 2178;
    int slowDownPivotHeight = 1000;
    double pitchPos = 0;
    double pitchStep = 5;
    double rollStep = 5;
    double rollPos = 0;
    double speedMultiplication = 1;
    private enum driveType {FIELD, ROBOT}
    private enum speed {FAST, SLOW}
    private enum slidePos {UP, DOWN, MOVING_TO_POSITION, JOYSTICK_CONTROL}
    private enum intakeDirection {FORWARD, BACKWARD}
    private enum intakePower {YES, NO}
    private enum pivotPos {DEPOSIT_FRONT, PICKUP, MOVING_TO_POSITION_90, JOYSTICK_CONTROL, NOT_MOVING, MOVING_TO_POSITION_0, MOVING_TO_POSITION_45, MOVING_TO_POSITION_70, MOVING_TO_POSITION_PICKUP, DEPOSIT_BACK,MIDDLE, FLAT}
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
        timer = new ElapsedTime();


        //state machines initialization
        drive = driveType.ROBOT;
        speedMultiplier = speed.FAST;
        slideUpOrDown = slidePos.DOWN;
        direction = intakeDirection.FORWARD;
        power = intakePower.NO;
        pivotStateMachine = pivotPos.FLAT;

        boolean dontmoveroll = false;

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
            dontmoveroll = false;
            driverControls.update();

            if (driverControls.driveTypeSwitch()){
                if (drive == driveType.ROBOT){
                    drive = driveType.FIELD;
                } else {
                    drive = driveType.ROBOT;
                }
            }
            if (driverControls.slowMode() || pivot.getCurrentPosition() > slowDownPivotHeight){
                if (speedMultiplier == speed.FAST || pivot.getCurrentPosition() > slowDownPivotHeight){
                    speedMultiplier = speed.SLOW;
                } else {
                    speedMultiplier = speed.FAST;
                }
            }
//slide controls soft stop
            maxAllowedExtension = arm.currentAllowedMaxExtensionLength();
            //Slide code
            if (driverControls.slidesFullyUp()){
                slideCode.goTo(maxAllowedExtension);
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (driverControls.slidesFullyDown()){
                slideCode.goTo(0);
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
            } else if (Math.abs(driverControls.slideMovement()) > 0 && slide.getTargetPosition() < PHYSICALMAXEXTENSION){
                slideCode.joystickControl(driverControls.slideMovement(), maxAllowedExtension);
                slideUpOrDown = slidePos.JOYSTICK_CONTROL;
            } else if (slide.getCurrentPosition() > maxAllowedExtension) {
                slideCode.goTo(maxAllowedExtension);
            } else if (slide.getCurrentPosition() < 0){
                slideCode.goTo(0);
            } else if (slideUpOrDown != slidePos.MOVING_TO_POSITION) {
                slideCode.holdPos();
            }
//elbow code
            if (driverControls.pivotParallel()){
                if (arm.doesSlideNeedToRetract(0) || arm.doesSlideNeedToRetract(pivot.getCurrentPosition())){
                    slideCode.goTo(Math.min(arm.currentAllowedMaxExtensionLength(), arm.currentAllowedMaxExtensionLength(0)));
                }
                else {
                    pivotCode.goTo(0);
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_0;
                }
            } else if (driverControls.pivotPerp()){
                if (arm.doesSlideNeedToRetract(topPivotPos) || arm.doesSlideNeedToRetract(pivot.getCurrentPosition())){
                    slideCode.goTo(Math.min(arm.currentAllowedMaxExtensionLength(), arm.currentAllowedMaxExtensionLength(90)));
                } else {
                    pivotCode.goTo(topPivotPos);
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_90;
                }
            } else if (driverControls.pivotJoystick() == 0 && (pivotStateMachine == pivotPos.JOYSTICK_CONTROL || pivotStateMachine == pivotPos.DEPOSIT_FRONT || pivotStateMachine == pivotPos.DEPOSIT_BACK || pivotStateMachine == pivotPos.PICKUP || pivotStateMachine == pivotPos.NOT_MOVING || pivotStateMachine == pivotPos.MIDDLE || pivotStateMachine == pivotPos.FLAT)){
                pivot.setPower(0);
            } else if (driverControls.pivotJoystick() < 0){
                if (arm.doesSlideNeedToRetract(pivot.getCurrentPosition() - 50) || arm.doesSlideNeedToRetract(pivot.getCurrentPosition())){
                    slideCode.goTo(Math.min(arm.currentAllowedMaxExtensionLength(), arm.currentAllowedMaxExtensionLength(pivot.getCurrentPosition() - 50)));
                } else {
                    pivotCode.pivotJoystick(pivot.getCurrentPosition(), driverControls.pivotJoystick());
                    pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
                }
            } else if (driverControls.pivotJoystick() > 0) {
                if (arm.doesSlideNeedToRetract(pivot.getCurrentPosition() + 50) || arm.doesSlideNeedToRetract(pivot.getCurrentPosition())){
                    slideCode.goTo(Math.min(arm.currentAllowedMaxExtensionLength(), arm.currentAllowedMaxExtensionLength(pivot.getCurrentPosition() + 50)));
                } else {
                    pivotCode.pivotJoystick(pivot.getCurrentPosition(), driverControls.pivotJoystick());
                    pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
                }
            }
            if (driverControls.submersibleIntakeReady()){

                telemetry.addLine("going to submersibleIntakeReady position");
                if (arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(-2)) || arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(pivot.getCurrentPosition()))){
                    slideCode.goTo(arm.currentAllowedMaxExtensionLength(pivotCode.degreesToTicks(-2)));
                } else {
                    pivotCode.goTo(pivotCode.degreesToTicks(-2));
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_45;
                    slideCode.goTo(407);
                }
                //slideCode.goTo(796);
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                pitchPos = -45;
                rollPos = 90;
                diffCode.setDifferentialPosition(pitchPos, rollPos);
                //timer.reset();
            }
            if (driverControls.drivingPos()){
                telemetry.addLine("going to drivingPos position");
                if (arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(45)) || arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(pivot.getCurrentPosition()))){
                    slideCode.goTo(arm.currentAllowedMaxExtensionLength(pivotCode.degreesToTicks(45)));
                } else {
                    pivotCode.goTo(pivotCode.degreesToTicks(45));
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_45;
                    slideCode.goTo(407);
                }
                slideUpOrDown = slidePos.MOVING_TO_POSITION;
                pitchPos = 90;
                rollPos = 90;
                diffCode.setDifferentialPosition(pitchPos, rollPos);
                //timer.reset();
                //dontmoveroll = true;
            }
            if (driverControls.acsent1Park()){
                telemetry.addLine("going to acsent1Park position");
                if (arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(45)) && pivotStateMachine != pivotPos.DEPOSIT_BACK){
                    slideCode.goTo(arm.currentAllowedMaxExtensionLength(pivotCode.degreesToTicks(90)));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else if (pivotStateMachine == pivotPos.DEPOSIT_BACK){
                    slideCode.goTo(slideCode.InchesToTicks(34));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else {
                    pivotCode.goTo(pivotCode.degreesToTicks(90));
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_90;
                }
                //slideCode.goTo(796);
                pitchPos = -90;
                rollPos = 90;
                diffCode.setDifferentialPosition(pitchPos, rollPos);
                //timer.reset();
                //dontmoveroll = true;
            }
            if (driverControls.depositReadyBackTopBucket()){
                telemetry.addLine("going to depositReadyBack position");
                if (arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(90)) && pivotStateMachine != pivotPos.DEPOSIT_BACK){
                    slideCode.goTo(arm.currentAllowedMaxExtensionLength(pivotCode.degreesToTicks(90)));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else if (pivotStateMachine == pivotPos.DEPOSIT_BACK){
                    slideCode.goTo(slideCode.InchesToTicks(34));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else {
                    pivotCode.goTo(pivotCode.degreesToTicks(90));
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_90;
                }
                rollPos = 90;
                pitchPos = 90;
                diffCode.setDifferentialPosition(pitchPos, rollPos);
                //timer.reset();
                //dontmoveroll = true;

            }
            if(driverControls.depositReadyFrontTopBucket()){
                telemetry.addLine("going to depositReadyUp position");
                if (arm.doesSlideNeedToRetract(pivotCode.degreesToTicks(70)) && pivotStateMachine != pivotPos.DEPOSIT_FRONT){
                    slideCode.goTo(arm.currentAllowedMaxExtensionLength(pivotCode.degreesToTicks(70)));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else if (pivotStateMachine == pivotPos.DEPOSIT_BACK){
                    slideCode.goTo(slideCode.InchesToTicks(41));
                    slideUpOrDown = slidePos.MOVING_TO_POSITION;
                } else {
                    pivotCode.goTo(pivotCode.degreesToTicks(70));
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION_70;
                }
                rollPos = 90;
                pitchPos = 90;
                diffCode.setDifferentialPosition(pitchPos, rollPos);
                //timer.reset();
                //dontmoveroll = true;
            }


            //State definitions
            if (driverControls.intakenewForward() > 0.5){
                direction = intakeDirection.FORWARD;
                power = intakePower.YES;
            } else if (driverControls.intakenewBackward() > 0.5/* || (blockColor == targetBlockColor.RED && activeIntakeSensor.red() >= 245 && activeIntakeSensor.green() <= 50) || (blockColor == targetBlockColor.BLUE && activeIntakeSensor.blue() >= 245) || (blockColor == targetBlockColor.RED && activeIntakeSensor.red() >= 245 && activeIntakeSensor.green() >= 245)*/){
                direction = intakeDirection.BACKWARD;
                power = intakePower.YES;
            } else {
                power = intakePower.NO;
            }
            if (pivotStateMachine == pivotPos.MOVING_TO_POSITION_45 && (pivot.getCurrentPosition() > pivotCode.degreesToTicks(44) && pivot.getCurrentPosition() < pivotCode.degreesToTicks(46))){
                pivotStateMachine = pivotPos.MIDDLE;
            }
            if (pivotStateMachine == pivotPos.MOVING_TO_POSITION_0 && (pivot.getCurrentPosition() > pivotCode.degreesToTicks(-1) && pivot.getCurrentPosition() < pivotCode.degreesToTicks(1))){
                pivotStateMachine = pivotPos.FLAT;
            }

            if (pivotStateMachine == pivotPos.MOVING_TO_POSITION_70 && (pivot.getCurrentPosition() > pivotCode.degreesToTicks(69) && pivot.getCurrentPosition() < pivotCode.degreesToTicks(71))){
                pivotStateMachine = pivotPos.DEPOSIT_FRONT;
            }

            if (pivotStateMachine == pivotPos.MOVING_TO_POSITION_90 && (pivot.getCurrentPosition() > pivotCode.degreesToTicks(89) && pivot.getCurrentPosition() < pivotCode.degreesToTicks(91))){
                pivotStateMachine = pivotPos.DEPOSIT_BACK;
            }

            if (pivotStateMachine == pivotPos.MOVING_TO_POSITION_PICKUP && (pivot.getCurrentPosition() > pivotCode.degreesToTicks(-16) && pivot.getCurrentPosition() < pivotCode.degreesToTicks(-15))){
                pivotStateMachine = pivotPos.MOVING_TO_POSITION_PICKUP;
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
                    driverCode.FieldCentricDrive(speedMultiplication, driverControls.resetIMU());
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

            /*if (timer.milliseconds() > 300 && !dontmoveroll){
                telemetry.addLine("Movingroll");
                diffCode.setDifferentialPosition(pitchPos, rollPos);
            }*/


            telemetry.addData("pitch", pitchPos);
            telemetry.addData("roll", rollPos);
            telemetry.addData("max slide height", maxAllowedExtension);
            telemetry.addData("elbow current draw", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("slide current draw", slide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
        }


    }

    private void initializeGamePads() {
        driverControls = new DriverControls(gamepad1, gamepad2);
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
        pitchPos = -90;
        rollPos = 90;
        //diffCode.setDifferentialPosition(pitchPos, rollPos);
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
        diffCode = new differential(left, right);
        diffCode.setDifferentialPosition(pitchPos, rollPos);

    }
}
