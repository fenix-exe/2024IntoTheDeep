package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.ConfigReader;
import org.firstinspires.ftc.teamcode.robot.RobotActions;
import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsytems.arm.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.arm.Slide;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.Differential;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffector;
import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserDirective;

import java.util.Set;

@TeleOp
public class TeleOPV4 extends LinearOpMode {
    DriveTrain driveTrain;
    ActiveIntake activeIntakeCode;
    Arm arm;
    EndEffector endEffector;
    DriverControls driverControls;
    DcMotorEx slide;
    DcMotorEx pivot;
    CRServo intake;
    ServoImplEx left;
    ServoImplEx right;
    differential diffCode;
    IMU imu;
    RevColorSensorV3 activeIntakeSensor;
    RevTouchSensor limitSwitch;
    double MaxSlideExtensionInches;
    int PHYSICALMAXEXTENSION = 2500;
    int maxAllowedExtension = 2500;
    //int lastTopHeight = 5000;
    int topPivotPos = 2178;
    double pitchPos = 0;
    double rollPos = 0;
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
        initializeEndEffector();
        //loadFromConfigFile("/sdcard/Download/TeleOp/PresetPositions.csv");



        //state machines initialization
        drive = driveType.ROBOT;
        speedMultiplier = speed.FAST;
        slideUpOrDown = slidePos.DOWN;
        direction = intakeDirection.FORWARD;
        power = intakePower.NO;
        pivotStateMachine = pivotPos.FLAT;


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
        RobotActions actions = new RobotActions();
        RobotCore.initialize(driverControls, driveTrain, arm, endEffector);
        waitForStart();



        while (opModeIsActive()){

            driverControls.update();

            Set<UserDirective> directive = driverControls.getUserIntents();

            RobotCore.updatePresetPositions(actions, directive);
            RobotCore.updateRobotActionsforArm(actions, directive);
            RobotCore.updateRobotActionsForEndEffector(actions, directive);
            RobotCore.updateRobotActionsforDriveTrain(actions, directive);

            String actions_list = actions.toString();
            telemetry.addData("Intent", directive);
            telemetry.addData("actions", actions_list);
            telemetry.addData("Slide extension", arm.getSlideExtension());
            telemetry.addData("Slide target position", arm.getSlideExtension());
            telemetry.addData("Slide limit", arm.getSlideMaxLengthIn42Inches(arm.getElbowAngleInTicks()));
            telemetry.addData("Elbow angle", arm.getElbowAngleInDegrees());
            telemetry.addData("Elbow target position", pivot.getTargetPosition());
            telemetry.addData("IMU yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("pitch", diffCode.returnPitch());
            telemetry.addData("roll", diffCode.returnRoll());
            telemetry.addData("slide current", slide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("elbow current", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Is the hall effect sensor triggered?", limitSwitch.isPressed());
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
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //home();

        Slide slideControl = new Slide(slide, PHYSICALMAXEXTENSION);
        Elbow elbow = new Elbow(pivot, limitSwitch, 2300);
        arm = new Arm(slideControl, elbow, PHYSICALMAXEXTENSION);

        slide.setTargetPosition(0);
        pivot.setTargetPosition(0);

    }
    private void initializeIntake(){
        intake = hardwareMap.get(CRServo.class, "intake");
        activeIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "activeIntakeSensor");
        activeIntakeCode = new ActiveIntake(intake, activeIntakeSensor);
    }
    private void initializeDifferential(){
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
        pitchPos = 0;
        rollPos = -90;
        diffCode = new differential(left,right);
        diffCode.setDifferentialPosition(pitchPos, rollPos);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffector(activeIntakeCode, diffCode);
    }
    private void loadFromConfigFile(String fileName){
        ConfigReader.readConfig(fileName);
    }
    private void home(){
        //Homing the elbow
        while (!limitSwitch.isPressed() && !isStopRequested()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
