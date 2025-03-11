package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.stateModels.PresetConfigUtil;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforREV;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.FrequencyCounter;
import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;
import org.firstinspires.ftc.teamcode.util.writeAuto;

import java.io.File;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

@Config
@TeleOp(name = "TeleOPPoseZapdos")
@Disabled
public class TeleOpPose extends LinearOpMode {
    MultipleTelemetry multiTelemetry;
    DriveTrain driveTrain;
    Arm arm;
    DriverControls driverControls;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    DcMotorEx pivot;
    DcMotorEx linearActuatorMotor;
    Servo clawServo;
    Servo pitch;
    Servo roll;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    LinearActuator linearActuator;
    IIMU imu;
    RevTouchSensor limitSwitch;
    RevTouchSensor homingSwitch;
    FrequencyCounter freqCounter;
    ElapsedTime matchTimer;
    double speedMultiplier;
    boolean USEREVIMU = true;
    boolean liftedLinearActuator = false;
    TelemetryPacket p;
    Gamepad gamepad1current;
    Gamepad gamepad2current;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    DecimalFormat df = new DecimalFormat("#.##");
    public static String filename = "test1";
    public static double x = -7;
    public static double y = 65;
    public static double heading = 0;




    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        initializeLinearActuator();
        PresetConfigUtil.loadPresetsFromConfig();
        //StateModelsZapdos.initialize(arm, wrist, claw, linearActuator, driverControls, null);
        DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        matchTimer = new ElapsedTime();

        writeAuto writer= new writeAuto(filename);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(x, y, Math.toRadians(heading)));
        gamepad1current = new Gamepad();
        gamepad2current = new Gamepad();

        gamepad1previous = new Gamepad();
        gamepad2previous = new Gamepad();

        gamepad1current.copy(gamepad1);
        gamepad2current.copy(gamepad2);

        df.setRoundingMode(RoundingMode.CEILING);




        waitForStart();
        matchTimer.reset();

        while (opModeIsActive()){

            driverControls.update();
            imu.update();

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
                speedMultiplier = RobotConstants.SLOW_SPEED;
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

            if (driverControls.resetEncoders()){
                arm.resetEncoders();
            }

            //checking if linear actuator should automatically go up


            //state models for preset positions
            /*StateModelsZapdos.presetPositionDriveStateModel(0,73,8);
            StateModelsZapdos.presetPositionIntakeStateModel(0,-90,-90,0,12,12);
            //StateModels.leaveSubmersibleStateModel(0,-90,2);
            //StateModels.presetPositionDepositStateModel(-30,0,75,33.5);
            StateModelsZapdos.presetPositionDepositStateModel(-45,0,75,28, 8);
            StateModelsZapdos.depositSampleIntoBucketStateModel(0,0,83,58,8);
            StateModelsZapdos.presetPositionGrabBlockFromOutsideStateModel(-90, 0,0,4,10, 58,0);
            StateModelsZapdos.presetPositionGrabBlockFromInsideStateModel(-90,0,-90,2,10,58,0);
            StateModelsZapdos.presetPositionPickupSpecimensStateModel(15,130,16.5,2.2, 77, 3, 7,90, 90);
            StateModelsZapdos.presetPositionDepositSpecimensStateModel(15,130,16.5,16,0);
            StateModelsZapdos.dropBlockAndMoveWristDown(-90,6);
            StateModelsZapdos.hang(0,0,9.5,6,60,28,90,45, 0,15);*/

            //telemetry
            multiTelemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            multiTelemetry.addData("Elbow Current", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Slide Length", arm.getSlideExtension());
            multiTelemetry.addData("Slide Current", leftSlide.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Wrist Pitch", pitch.getPosition());
            multiTelemetry.addData("Wrist Roll", roll.getPosition());
            multiTelemetry.addData("imu", Math.toDegrees(imu.getYaw()));
            /*multiTelemetry.addData("Dropping Block State Model", StateModelsFawkes.enterIntakePositionStates);
            multiTelemetry.addData("Deposit State Model", StateModelsFawkes.depositBackPresetState);
            multiTelemetry.addData("Intake State Model", StateModelsFawkes.intakePresetState);
            multiTelemetry.addData("Y Cycle", StateModelsFawkes.depositCycle);
            multiTelemetry.addData("At intake position?", StateModelsFawkes.intakePosition);
            multiTelemetry.addData("Specimen Pickup State", StateModelsFawkes.pickupSpecimenState);
            multiTelemetry.addData("Block Pickup Type", StateModelsFawkes.blockPickupType);*/
            multiTelemetry.addData("Strategy", driverControls.getGameStrategyMode());
            multiTelemetry.addData("Driving Mode", DriveTrain.driveType);
            multiTelemetry.addData("speed multipler", speedMultiplier);
            multiTelemetry.update();

            //logging
            logDriveTrain();
            logArm();
            logEndEffector();
            logStateModels();
            logButtonPressed();

            if (gamepad1current.dpad_up && !gamepad1previous.dpad_up) {
                writer.writeToFile(Double.parseDouble(df.format(drive.pose.position.x)), Double.parseDouble(df.format(drive.pose.position.y)),Double.parseDouble(df.format(drive.pose.heading.toDouble())),Double.parseDouble(df.format(arm.getElbowAngleInDegrees())),Double.parseDouble(df.format(arm.getSlideExtension())), pitch.getPosition(), roll.getPosition(), clawServo.getPosition());
            }
            if (gamepad1.left_bumper) {
                Arrays.stream(new File("/sdcard/Download/autoLogger").listFiles()).forEach(File::delete);
            }

        }
    }

    private void initializeGamePads() {
        driverControls = new DriverControls(gamepad1, gamepad2, 1);
        driverControls.setGameStrategyMode(DriverControls.scoringType.SPECIMEN);
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
        if (USEREVIMU){
            IMU revIMU = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            revIMU.initialize(parameters);
            //imu.resetYaw();
            imu = new IMUforREV(revIMU);
        } else {
            GoBildaPinpointDriver pinpointIMU = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
            //imu = new IMUforPinpoint(pinpointIMU);
        }

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, imu);
    }
    private void initializeArmAndHome(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmConstants.MAXSLIDEEXTENSIONLENGTHINCHES = 19;


        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide slideControl = new Slide(leftSlide, rightSlide, homingSwitch);
        Elbow elbow = new Elbow(pivot, limitSwitch,90);
        arm = new Arm(slideControl, elbow);

        leftSlide.setTargetPosition(0);
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
        wrist = new Wrist(pitch, roll);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffectorV2(wrist, claw);
    }
    private void initializeLinearActuator(){
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        linearActuatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RevTouchSensor actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "actuator switch");

        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

        linearActuator.resetEncoders();
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
        //LoggerUtil.debug("stateModels", StateModelsFawkes.getDebugString());
    }
    private void logButtonPressed(){
        LoggerUtil.debug("buttonPresses", String.valueOf(driverControls.slideMovement()));
    }


}
