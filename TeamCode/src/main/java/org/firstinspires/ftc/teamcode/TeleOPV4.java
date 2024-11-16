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
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotActions;
import org.firstinspires.ftc.teamcode.robot.RobotActivityState;
import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.robot.RobotPhysicalState;
import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmActionList;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsytems.arm.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.arm.Slide;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainActionList;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorActionList;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorState;
import org.firstinspires.ftc.teamcode.subsytems.modules.RobotArm;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.driveCode;
import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserIntent;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainState;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

import java.util.List;
import java.util.Set;

@TeleOp
public class TeleOPV4 extends LinearOpMode {
    DriveTrain driveTrain;
    activeIntake activeIntakeCode;
    Arm arm;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DriverControls driverControls;
    activeIntake intakeCode;
    /*Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad gamepad1current;
    Gamepad gamepad2current;*/

    DcMotorEx slide;
    DcMotorEx pivot;
    CRServo intake;
    Servo left;
    Servo right;
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
        //timer = new ElapsedTime();


        //state machines initialization
        drive = driveType.ROBOT;
        speedMultiplier = speed.FAST;
        slideUpOrDown = slidePos.DOWN;
        direction = intakeDirection.FORWARD;
        power = intakePower.NO;
        pivotStateMachine = pivotPos.FLAT;

        boolean dontmoveroll = false;

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

        /*DriveTrainState driveActivityState = DriveTrainState.IDLE;
        ArmState armActivityState = ArmState.IDLE;
        EndEffectorState endEffectorActivityState = EndEffectorState.IDLE;*/

        /*while (opModeIsActive()){

            Set<UserIntent> intent = driverControls.getUserIntents();
            Set<RobotActivityState> currentActivityState = RobotCore.getCurrentActivityState();  // This is the overall state of the robot, not the subsystem

            RobotPhysicalState currentPhysicalState = RobotCore.getRobotPhysicalState();  // arm angle, slide len, differential angle, intake status

            DriveTrainActionList driveTrainActions = RobotCore.getStepsForUserIntentForDriveTrain(intent, currentPhysicalState, currentActivityState);
            ArmActionList armActions = RobotCore.getStepsForUserIntentForArm(intent, currentPhysicalState, currentActivityState);
            EndEffectorActionList endEffector = RobotCore.getStepsForUserIntentForEndEffector(intent, currentPhysicalState, currentActivityState);

            driveActivityState = driveTrainActions.execute();
            armActivityState =armActions.execute();
            endEffectorActivityState =endEffector.execute();
            RobotCore.setCurrentActivityState(driveActivityState, armActivityState, endEffectorActivityState, intent);
        }*/

        RobotActions actions = new RobotActions();
        RobotCore.initialize(driverControls, driveTrain, arm);
        while (opModeIsActive()){

            driverControls.update();

            Set<UserIntent> intent = driverControls.getUserIntents();

            RobotCore.updateRobotActionsforArm(actions, intent);
            //RobotCore.updateActiveActionsforEndEffector(actions, intent);
            RobotCore.updateRobotActionsforDriveTrain(actions, intent);

            String actions_list = actions.toString();
            telemetry.addData("Intent", intent);
            telemetry.addData("actions", actions_list);
            telemetry.addData("Arm extension", arm.getSlideExtension());
            telemetry.addData("Slide limit", arm.getSlideMaxLengthIn42Inches(arm.getElbowAngleInTicks()));
            telemetry.addData("Elbow target position", pivot.getTargetPosition());
            telemetry.update();

            actions.execute();
            actions.removeCompleteAndCancelled();

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

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, imu, telemetry);
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
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //slideCode = new slideCodeFunctions(slide);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, topPivotPos);
        Slide slideControl = new Slide(slide, PHYSICALMAXEXTENSION);
        Elbow elbow = new Elbow(pivot, 2100);
        arm = new Arm(slideControl, elbow, PHYSICALMAXEXTENSION);
    }
    private void initializeIntake(){
        intake = hardwareMap.get(CRServo.class, "intake");
        //activeIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "activeIntakeSensor");
        activeIntakeCode = new activeIntake(intake);
    }
    private void initializeDifferential(){
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        pitchPos = -90;
        rollPos = 90;
        //diffCode.setDifferentialPosition(pitchPos, rollPos);
    }
}
