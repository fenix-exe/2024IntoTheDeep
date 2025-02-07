package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.stateModels.PresetConfigUtil;
import org.firstinspires.ftc.teamcode.teleop.stateModels.StateModelsZapdos;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforREV;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.localization.Localization;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.FrequencyCounter;
import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@TeleOp
public class TeleOpV5SampleZapdos extends LinearOpMode {
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
    ColorSensor color;
    LinearActuator linearActuator;
    Localization localization;
    IIMU imu;
    RevTouchSensor limitSwitch;
    RevTouchSensor homingSwitch;
    RevColorSensorV3 colorSensor;
    ElapsedTime matchTimer;
    ElapsedTime debounceTimer;
    FrequencyCounter freqCounter;
    double speedMultiplier;
    boolean liftedLinearActuator = false;
    boolean touchSensorPressedLastLoop = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        initializeLinearActuator();
        PresetConfigUtil.loadPresetsFromConfig();
        StateModelsZapdos.initialize(arm, wrist, claw, linearActuator, driverControls, color);
        DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        matchTimer = new ElapsedTime();


        waitForStart();
        wrist.presetPosition(0,0);
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
                multiTelemetry.addLine("MANUAL CONTROL MOVE");
                multiTelemetry.addData("Slide Movement", driverControls.slideMovement());
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

            if (driverControls.resetEncoders()){
                arm.resetEncoders();
            }

            if (arm.isSlideTouchSensorPressed() && debounceTimer.milliseconds() >2000){
                arm.resetSlideEncoders();
                debounceTimer.reset();
            }

            //checking if linear actuator should automatically go up
            if (driverControls.linearActuatorUp()){
                linearActuator.goToTargetPositionInches(9.5);
            }
            if (driverControls.linearActuatorDown()){
                linearActuator.goToTargetPositionInches(linearActuator.getLinearActuatorPositionInches() - 0.25);
            } /*else {
                linearActuator.goToTargetPositionInches(linearActuator.getLinearActuatorPositionInches());
            }*/
            //matchTimer.seconds() > 100 ||

            //state models for preset positions
            StateModelsZapdos.presetPositionDriveStateModel(0,58,8);
            StateModelsZapdos.presetPositionIntakeStateModel(-40,-60,-105,-3,0,12);
            //StateModels.leaveSubmersibleStateModel(0,-90,2);
            //StateModels.presetPositionDepositStateModel(-30,0,75,33.5);
            //StateModelsZapdos.presetPositionDepositFrontStateModel(-100,-30,83,28, 8);
            StateModelsZapdos.presetPositionDepositFrontStateModel(100,0,92,28, 6);
            StateModelsZapdos.depositSampleIntoBucketStateModel(-105,-3,80,0,12);
            StateModelsZapdos.presetPositionGrabBlockFromOutsideStateModel(-105,0,0,0.8,0.8,58,0);
            StateModelsZapdos.presetPositionPickupSpecimensStateModel(0,-90,0,4.75, 31, 3,9, 30, -90);
            StateModelsZapdos.presetPositionDepositSpecimensStateModel(0,-90,28,0);
            StateModelsZapdos.dropBlockAndMoveWristDown(-105, 1.9);
            StateModelsZapdos.depositSampleIntoObservationZone(3,0,16,-105,-3);
            StateModelsZapdos.hang(5,0,9.5,5.75,83,26,103,45,3,15);
 
            //telemetry
            multiTelemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            /*multiTelemetry.addData("Elbow Current", pivot.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Elbow at Target Angle?", Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE);
            multiTelemetry.addData("Slide Length", arm.getSlideExtension());
            multiTelemetry.addData("Slide Encoder Left", leftSlide.getCurrentPosition());
            multiTelemetry.addData("Slide Encoder Right", rightSlide.getCurrentPosition());
            multiTelemetry.addData("Slide Current Left", leftSlide.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Slide Current Right", rightSlide.getCurrent(CurrentUnit.MILLIAMPS));
            multiTelemetry.addData("Slide Target Left", leftSlide.getTargetPosition());
            multiTelemetry.addData("Slide Target Right", rightSlide.getTargetPosition());
            multiTelemetry.addData("Wrist Pitch", wrist.getPitchAngle());
            multiTelemetry.addData("Wrist Roll", wrist.getRollAngle());
            multiTelemetry.addData("Pitch Servo Pos", pitch.getPosition());
            multiTelemetry.addData("Roll Servo Pos", roll.getPosition());
            multiTelemetry.addData("IMU", Math.toDegrees(imu.getYaw()));
            multiTelemetry.addData("Dropping Block State Model", StateModelsZapdos.enterIntakePositionStates);
            multiTelemetry.addData("Deposit State Model", StateModelsZapdos.depositBackPresetState);
            multiTelemetry.addData("Intake State Model", StateModelsZapdos.intakePresetState);
            multiTelemetry.addData("Y Cycle", StateModelsZapdos.depositCycle);
            multiTelemetry.addData("At intake position?", StateModelsZapdos.intakePosition);
            multiTelemetry.addData("Specimen Pickup State", StateModelsZapdos.pickupSpecimenState);
            multiTelemetry.addData("Sample Intake State", StateModelsZapdos.grabBlockFromOutsidePresetState);
            multiTelemetry.addData("Block Pickup Type", StateModelsZapdos.blockPickupType);
            multiTelemetry.addData("Strategy", driverControls.getGameStrategyMode());
            multiTelemetry.addData("Driving Mode", DriveTrain.driveType);
            multiTelemetry.addData("Speed Multiplier", speedMultiplier);
            multiTelemetry.addData("linear actuator", linearActuator.getLinearActuatorPositionInches());
            multiTelemetry.addData("Claw Distance in CM", colorSensor.getDistance(DistanceUnit.CM));
            multiTelemetry.addData("Claw Alpha", colorSensor.alpha());
            multiTelemetry.addData("Red", colorSensor.red());
            multiTelemetry.addData("Blue", colorSensor.blue());
            multiTelemetry.addData("Green", colorSensor.green());*/
            multiTelemetry.update();

            //logging
            logDriveTrain();
            logArm();
            logEndEffector();
            logStateModels();
            logButtonPressed();

            touchSensorPressedLastLoop = arm.isSlideTouchSensorPressed();
        }
    }

    private void initializeGamePads() {
        driverControls = new DriverControls(gamepad1, gamepad2, 1);
        driverControls.setGameStrategyMode(DriverControls.scoringType.SAMPLE);
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
        IMU revIMU = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        revIMU.initialize(parameters);
        //imu.resetYaw();
        imu = new IMUforREV(revIMU);
        GoBildaPinpointDriverRR pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

        localization = new Localization(pinpoint, revIMU);

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, imu, telemetry);
    }
    private void initializeArmAndHome(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        debounceTimer = new ElapsedTime();
        debounceTimer.reset();

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setTargetPositionTolerance(0);
        rightSlide.setTargetPositionTolerance(0);
        pivot.setTargetPositionTolerance(0);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmConstants.MAXSLIDEEXTENSIONLENGTHINCHES = 14;

        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide slideControl = new Slide(leftSlide, rightSlide, homingSwitch);
        Elbow elbow = new Elbow(pivot, limitSwitch, 100);
        arm = new Arm(slideControl, elbow);

        leftSlide.setTargetPosition(0);
        pivot.setTargetPosition(0);

    }
    private void initializeIntake(){
        clawServo = hardwareMap.get(Servo.class, "claw");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        claw = new Claw(clawServo);
        color = new ColorSensor(colorSensor);

    }
    private void initializeDifferential(){
        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        roll.setDirection(Servo.Direction.REVERSE);
        wrist = new Wrist(pitch, roll);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffectorV2(wrist, claw);
    }
    private void initializeLinearActuator(){
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        RevTouchSensor actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");

        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

        linearActuator.goToTargetPositionInches(0);
    }

    private void logDriveTrain(){
        HashMap driveTrainInfo = driveTrain.getDebugInfo();
        HashMap localizationInfo = localization.getDebugInfo();
        ArrayList values = new ArrayList();
        values.add(driveTrainInfo.get("FL Power"));
        values.add(driveTrainInfo.get("BL Power"));
        values.add(driveTrainInfo.get("FR Power"));
        values.add(driveTrainInfo.get("BR Power"));
        values.add(driveTrainInfo.get("FL Current"));
        values.add(driveTrainInfo.get("BL Current"));
        values.add(driveTrainInfo.get("FR Current"));
        values.add(driveTrainInfo.get("BR Current"));
        values.add(localizationInfo.get("x"));
        values.add(localizationInfo.get("y"));
        values.add(localizationInfo.get("h"));
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
        LoggerUtil.debug("stateModels", StateModelsZapdos.getDebugString());
    }
    private void logButtonPressed(){
        LoggerUtil.debug("buttonPresses", String.valueOf(driverControls.slideMovement()));
    }
}
