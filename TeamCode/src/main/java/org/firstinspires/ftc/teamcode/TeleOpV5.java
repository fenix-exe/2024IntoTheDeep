package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.robot.ConfigUtil;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.util.FrequencyCounter;

public class TeleOpV5 extends LinearOpMode {
    DriveTrain driveTrain;
    ActiveIntake activeIntakeCode;
    Arm arm;
    DriverControls driverControls;
    DcMotorEx slide;
    DcMotorEx pivot;
    Servo clawServo;
    Servo pitch;
    Servo roll;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    IMU imu;
    RevColorSensorV3 activeIntakeSensor;
    RevTouchSensor limitSwitch;
    FrequencyCounter freqCounter;
    int PHYSICALMAXEXTENSION = 2500;
    double SLIDE_TOLERANCE = 1;
    double ELBOW_TOLERANCE = 3;

    public enum presetDriveState {START, MOVING_WRIST, MOVING_SLIDE, MOVING_ELBOW}
    public enum presetIntakeState {START, MOVING_WRIST, MOVING_SLIDE, MOVING_ELBOW}
    presetDriveState drivePresetState;
    presetIntakeState intakePresetState;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        ConfigUtil.loadPresetsFromConfig();
        driveTrain.driveType = driveTrain.driveType.ROBOT_CENTRIC;
        drivePresetState = presetDriveState.START;
        intakePresetState = presetIntakeState.START;

        waitForStart();

        while (opModeIsActive()){
            //driving code
            if (driverControls.driveTypeSwitch()){
                if (driveTrain.driveType == driveTrain.driveType.ROBOT_CENTRIC){
                    driveTrain.driveType = driveTrain.driveType.FIELD_CENTRIC;
                } else{
                    driveTrain.driveType = driveTrain.driveType.ROBOT_CENTRIC;
                }

            }

            if (driverControls.resetIMU()){
                driveTrain.resetIMU();
            }

            switch (driveTrain.driveType) {
                case ROBOT_CENTRIC:
                    driveTrain.RobotCentric_Drive(1);
                    break;
                case FIELD_CENTRIC:
                    driveTrain.FieldCentricDrive(1);
                    break;
            }

            //manual control
            if (driverControls.slideMovement() > 0){
                arm.moveSlide(driverControls.slideMovement(), driverControls.removeArmRules());
            }
            if (driverControls.pivotJoystick() > 0){
                arm.moveElbow(driverControls.pivotJoystick());
            }

            presetPositionDriveStateModel(-0.5,-0.5,45,0);
            presetPositionIntakeStateModel(0,0,0,2);


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
        //imu.resetYaw();

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

        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide slideControl = new Slide(slide, PHYSICALMAXEXTENSION);
        Elbow elbow = new Elbow(pivot, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2300);
        arm = new Arm(slideControl, elbow, PHYSICALMAXEXTENSION);

        slide.setTargetPosition(0);
        pivot.setTargetPosition(0);

    }
    private void initializeIntake(){
        clawServo = hardwareMap.get(Servo.class, "claw");
        claw = new Claw(clawServo);

    }
    private void initializeDifferential(){
        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        wrist = new Wrist(pitch, roll);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffectorV2(wrist, claw);
    }
    private void presetPositionDriveStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (drivePresetState){
            case START:
                if (driverControls.drivingPos()){
                    timer = new ElapsedTime();
                    timer.reset();
                    endEffector.goToPresetPosition(pitch,roll);
                    drivePresetState = presetDriveState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    drivePresetState = presetDriveState.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    drivePresetState = presetDriveState.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - slideLength < SLIDE_TOLERANCE) {
                    arm.moveElbowToAngle(elbowAngle);
                    drivePresetState = presetDriveState.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    drivePresetState = presetDriveState.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - elbowAngle) < ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    drivePresetState = presetDriveState.START;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    drivePresetState = presetDriveState.START;
                }
                break;
        }
    }

    private void presetPositionIntakeStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (intakePresetState){
            case START:
                if (driverControls.drivingPos()){
                    timer = new ElapsedTime();
                    timer.reset();
                    endEffector.goToPresetPosition(pitch,roll);
                    drivePresetState = presetDriveState.START;
                    intakePresetState = presetIntakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    intakePresetState = presetIntakeState.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    intakePresetState = presetIntakeState.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - slideLength < SLIDE_TOLERANCE) {
                    arm.moveElbowToAngle(elbowAngle);
                    intakePresetState = presetIntakeState.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    intakePresetState = presetIntakeState.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - elbowAngle) < ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    intakePresetState = presetIntakeState.START;
                }
                if (driverControls.escapePresets()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    arm.moveElbowToAngle(arm.getElbowAngleInDegrees());
                    intakePresetState = presetIntakeState.START;
                }
                break;
        }
    }

}
