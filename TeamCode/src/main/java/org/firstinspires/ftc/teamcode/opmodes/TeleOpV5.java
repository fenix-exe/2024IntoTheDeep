package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.stateModels.PresetConfigUtil;
import org.firstinspires.ftc.teamcode.stateModels.StateModels;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.util.FrequencyCounter;
import org.firstinspires.ftc.teamcode.util.LoggerUtil;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@TeleOp
public class TeleOpV5 extends LinearOpMode {
    MultipleTelemetry multiTelemetry;
    DriveTrain driveTrain;
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
    RevTouchSensor limitSwitch;
    FrequencyCounter freqCounter;
    double speedMultiplier;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        PresetConfigUtil.loadPresetsFromConfig();
        StateModels.initialize(arm, wrist, claw, driverControls);
        DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        wrist.presetPosition(0,0);

        while (opModeIsActive()){

            driverControls.update();

            //driving code
            if (driverControls.driveTypeSwitch()){
                if (DriveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
                    DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
                } else{
                    DriveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
                }

            }

            if (driverControls.resetIMU()){
                driveTrain.resetIMU();
            }

            //speed adjustments
            if (driverControls.microDriveAdjustments()){
                speedMultiplier = RobotConstants.EXTRA_SLOW;
            } /*else if (arm.getElbowAngleInDegrees() < RobotConstants.ELBOW_SLOW_DOWN_DRIVETRAIN_BOTTOM_ANGLE) {
                speedMultiplier = RobotConstants.NORMAL_SPEED;
            } else if (arm.getElbowAngleInDegrees() > RobotConstants.ELBOW_SLOW_DOWN_DRIVETRAIN_TOP_ANGLE) {
                speedMultiplier = RobotConstants.EXTRA_SLOW;
            }*/ else {
                speedMultiplier = RobotConstants.NORMAL_SPEED;
            }


            switch (DriveTrain.driveType) {
                case ROBOT_CENTRIC:
                    driveTrain.RobotCentric_Drive(speedMultiplier);
                    break;
                case FIELD_CENTRIC:
                    driveTrain.FieldCentricDrive(speedMultiplier);
                    break;
            }

            //manual control for arm
            if (Math.abs(driverControls.slideMovement()) > 0){
                arm.moveSlide(driverControls.slideMovement(), driverControls.removeArmRules());
            } else if (driverControls.slideStopped()){
                arm.holdSlide();
            }
            if (Math.abs(driverControls.pivotJoystick()) > 0){
                arm.moveElbow(driverControls.pivotJoystick());
            } else if (driverControls.pivotManualStopped()){
                arm.holdElbow();
            }

            //manual control for wrist
            if (driverControls.diffDown()){
                wrist.manualControlPitch(-15);
                multiTelemetry.addLine("Wrist Down");
            }
            if (driverControls.diffUp()){
                wrist.manualControlPitch(15);
                multiTelemetry.addLine("Wrist Up");
            }
            if (driverControls.diffLeft()){
                wrist.manualControlRoll(-30);
                multiTelemetry.addLine("Wrist Left");
            }
            if (driverControls.diffRight()){
                wrist.manualControlRoll(30);
                multiTelemetry.addLine("Wrist Right");
            }

            //manual control for claw
            if (driverControls.openClaw()){
                claw.openClaw();
            }
            if (driverControls.closeClaw()){
                claw.closeClaw();
            }

            //switching modes
            if(driverControls.switchStrategy()) {
                if (driverControls.getGameStrategyMode() == DriverControls.scoringType.SAMPLE){
                    driverControls.setGameStrategyMode(DriverControls.scoringType.SPECIMEN);
                } else {
                    driverControls.setGameStrategyMode(DriverControls.scoringType.SAMPLE);
                }
            }


            //state models for preset positions
            StateModels.presetPositionDriveStateModel(0,58,8);
            StateModels.presetPositionIntakeStateModel(0,-90,-90,0,12,12);
            //StateModels.leaveSubmersibleStateModel(0,-90,2);
            StateModels.presetPositionDepositStateModel(-30,0,73,30.5);
            StateModels.presetPositionDepositBackStateModel(75,0,85,24, 8);
            StateModels.depositSampleIntoBucketStateModel(0,0,58,8);
            StateModels.presetPositionGrabBlockFromOutsideStateModel(-90, 0,0,4,10, 58,0);
            StateModels.presetPositionGrabBlockFromInsideStateModel(-90,0,-90,2,10,58,0);
            StateModels.presetPositionPickupSpecimensStateModel(-10,90,25,0, 85);
            StateModels.presetPositionDepositSpecimensStateModel(90,90,85,58,3,13);
            StateModels.dropBlockAndMoveWristDown(-90);

            //telemetry
            multiTelemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            multiTelemetry.addData("Elbow Current", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Slide Length", arm.getSlideExtension());
            multiTelemetry.addData("Slide Current", slide.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Wrist Pitch", pitch.getPosition());
            multiTelemetry.addData("Wrist Roll", roll.getPosition());
            multiTelemetry.addData("Dropping Block State Model", StateModels.enterIntakePositionStates);
            multiTelemetry.addData("Deposit State Model", StateModels.depositBackPresetState);
            multiTelemetry.addData("Intake State Model", StateModels.intakePresetState);
            multiTelemetry.addData("Y Cycle", StateModels.depositCycle);
            multiTelemetry.addData("At intake position?", StateModels.intakePosition);
            multiTelemetry.addData("Block Pickup Type", StateModels.blockPickupType);
            multiTelemetry.addData("Strategy", driverControls.getGameStrategyMode());
            multiTelemetry.update();

            //logging
            logDriveTrain();
            logArm();
            logEndEffector();
            logStateModels();
            logButtonPressed();

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

        Slide slideControl = new Slide(slide);
        Elbow elbow = new Elbow(pivot, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2300);
        arm = new Arm(slideControl, elbow);

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
        pitch.setDirection(Servo.Direction.REVERSE);
        roll.setDirection(Servo.Direction.REVERSE);
        wrist = new Wrist(pitch, roll);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffectorV2(wrist, claw);
    }

    private void logDriveTrain(){
        HashMap driveTrainInfo = driveTrain.getDebugInfo();
        ArrayList values = new ArrayList();
        values.add(driveTrainInfo.get("FL Power"));
        values.add(driveTrainInfo.get("BL Power"));
        values.add(driveTrainInfo.get("FR Power"));
        values.add(driveTrainInfo.get("BR Power"));
        values.add(driveTrainInfo.get("FL Current"));
        values.add(driveTrainInfo.get("BL Current"));
        values.add(driveTrainInfo.get("FR Current"));
        values.add(driveTrainInfo.get("BR Current"));
        String debugString = String.join(",", values);
        LoggerUtil.debug("drivetrain", debugString);
    }
    private void logArm(){
        HashMap armInfo = arm.getDebugInfo();
        ArrayList values = new ArrayList();
        values.add(armInfo.get("Slide Extension"));
        values.add(armInfo.get("Slide Limit"));
        values.add(armInfo.get("Slide Power"));
        values.add(armInfo.get("Slide Current"));
        values.add(armInfo.get("Elbow Angle"));
        values.add(armInfo.get("Elbow Power"));
        values.add(armInfo.get("Elbow Current"));
        String debugString = String.join(",", values);
        LoggerUtil.debug("arm", debugString);
    }
    private void logEndEffector(){
        HashMap endEffectorInfo = endEffector.getDebugInfo();
        ArrayList values = new ArrayList();
        values.add(endEffectorInfo.get("Pitch Angle"));
        values.add(endEffectorInfo.get("Roll Angle"));
        values.add(endEffectorInfo.get("Claw Servo Position"));
        String debugString = String.join(",", values);
        LoggerUtil.debug("endEffector", debugString);
    }
    private void logStateModels(){
        LoggerUtil.debug("stateModels", StateModels.getDebugString());
    }
    private void logButtonPressed(){
        LoggerUtil.debug("buttonPresses", String.valueOf(driverControls.slideMovement()));
    }


}
