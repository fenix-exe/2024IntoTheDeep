package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.stateModels.GrabSpecimenStateTransition;
import org.firstinspires.ftc.teamcode.teleop.stateModels.StateModelParameters;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.stateModels.PresetConfigUtil;
import org.firstinspires.ftc.teamcode.teleop.stateModels.ResetSlideEncoderStateModel;
import org.firstinspires.ftc.teamcode.teleop.stateModels.FSMManager;
import org.firstinspires.ftc.teamcode.teleop.stateModels.RobotState;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforPinpoint;
import org.firstinspires.ftc.teamcode.teleop.subsytems.LED.ILED;
import org.firstinspires.ftc.teamcode.teleop.subsytems.LED.LED;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrainAuto;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.localization.Localization;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;
import org.firstinspires.ftc.teamcode.teleop.util.FrequencyCounter;
import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import page.j5155.expressway.ftc.actions.ActionRunner;

@Config
@TeleOp
public class TeleOpBlue extends LinearOpMode {
    MultipleTelemetry multiTelemetry;
    IDriveTrain driveTrain;
    Arm arm;
    DriveControlMap driverControls;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    DcMotorEx pivot;
    DcMotorEx linearActuatorMotor;
    CRServoImplEx leftRoller;
    CRServoImplEx rightRoller;
    Servo pitch;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    IIntake intake;
    ColorSensor color;
    LED led;
    LinearActuator linearActuator;
    Localization localization;
    IIMU imu;
    RevTouchSensor elbowSwitch;
    RevTouchSensor slideSwitch;
    RevColorSensorV3 colorSensor;
    ElapsedTime matchTimer;
    ElapsedTime debounceTimer;
    FrequencyCounter freqCounter;
    double speedMultiplier;
    public static boolean enableLogging=false;
    protected static Alliance alliance = Alliance.BLUE;
    boolean colorSensorDetected;
    boolean checkColorSensor=true;

    @Override
    public void runOpMode() throws InterruptedException {
        //enable manual bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //initialization
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        initializeLinearActuator();
        initializeLED();
        int presetsRead = PresetConfigUtil.loadPresetsFromConfig();
        colorSensorDetected = color.isConnected();
        if (!colorSensorDetected){
            telemetry.addLine("COLOR SENSOR NOT DETECTED");
        } else {
            telemetry.addLine("COLOR SENSOR DETECTED");
        }
        initializeStateModels();
        ResetSlideEncoderStateModel.initialize(arm);
        //drivers prefer field centric so that is our default mode
        DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        matchTimer = new ElapsedTime();
        freqCounter = new FrequencyCounter();

        telemetry.addData("Presets Read", presetsRead);


        telemetry.update();

        waitForStart();
        matchTimer.reset();

        while (opModeIsActive()){
            //clear cache for bulk reads
            //IMPORTANT!!!!!!!!!!!!!!!! bc we are using manual bulk read mode
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //tracking loop cycle times, allowing us to know how many times our main while loop executes every second
            freqCounter.count();

            //updates for imu and button presses
            driverControls.update();
            imu.update();

            //switching drive modes
            if (driverControls.driveTypeSwitch()){
                driveTrain.setDriveType(IDriveTrain.DriveType.FIELD_CENTRIC);
            }

            //imu reset
            if (driverControls.resetIMU()){
                driveTrain.resetIMU();
            }

            //speed adjustments
            if (driverControls.slowMode() || FSMManager.robotState == RobotState.READY_TO_ENTER_SUBMERSIBLE || FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE){
                speedMultiplier = RobotConstants.SLOW_SPEED;
            } else {
                speedMultiplier = RobotConstants.NORMAL_SPEED;
            }
            driveTrain.setMaxPower(speedMultiplier);

            //manual move of the drivetrain
            driveTrain.Move(driverControls.forwardDrive(), driverControls.strafeDrive(), driverControls.turnDrive());

            //manual control for slide
            if (Math.abs(driverControls.slideMovement()) > 0){
                arm.moveSlide(driverControls.slideMovement(), driverControls.removeArmRules());
            } else if (driverControls.slideStopped()){
                //prevents slides from moving after the drivers let go of the joystick
                arm.holdSlide();
                //arm.moveSlideToLength(arm.getSlideExtension());
            }

            //manual control for elbow
            if (Math.abs(driverControls.pivotJoystick()) > 0){
                telemetry.addData("Elbow Movement",arm.moveElbow(driverControls.pivotJoystick()));
            } else if (driverControls.pivotManualStopped()){
                //prevents elbow from moving after the drivers let go of the joystick
                arm.holdElbow();
            }

            //Manual control for wrist up
            if (driverControls.diffUp()){
                wrist.manualControlPitch(0.005);
            }

            //manual control for wrist down
            if (driverControls.diffDown()){
                wrist.manualControlPitch(-0.005);
            }


            //manual control for claw
            if (driverControls.outtake()){
                if (!(intake.getIntakeDirection() == IIntake.IntakeDirection.BACKWARD)) {
                    intake.outtake();
                } else{
                    intake.stop();
                }
            } else if (driverControls.intake()){
                if (!(intake.getIntakeDirection() == IIntake.IntakeDirection.FORWARD)) {
                    intake.intake();
                } else{
                    intake.stop();
                }
            }

            //run touch sensor fsm for resetting slides
            ResetSlideEncoderStateModel.execute();

            //linear actuator code for driver control outside of state models
            if (driverControls.linearActuatorUp()){
                if (driverControls.slowMode()){
                    //for manual movements
                    double pos = linearActuator.getLinearActuatorPositionInches() + 1;
                    linearActuator.goToTargetPositionInches(pos);
                } else {
                    //preset positions
                    linearActuator.goToTargetPositionInches(9.5);
                }
            }
            if (driverControls.linearActuatorDown()){
                if (driverControls.slowMode()){
                    //for manual movements
                    if (!linearActuator.getLimitSwitchState()){
                        //prevents the linear actuator from driving into the ground
                        linearActuator.goToTargetPositionInches(Math.max(linearActuator.getLinearActuatorPositionInches() - 0.25,0.4));
                    }
                } else {
                    //preset position
                    linearActuator.goToTargetPositionInches(5.75);
                }

            }

            //led blinking
            updateLED();

            if (driverControls.escapePresets()){
                arm.holdArm();
                FSMManager.stopTransitions();
                FSMManager.setRobotStateToStart();
            }

            //check color sensor based on robot state
            if ((FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE || FSMManager.robotState == RobotState.READY_TO_GRAB_SPECIMEN) && checkColorSensor && colorSensorDetected){
                if (!color.isConnected()){
                    FSMManager.updateBasedOnColorSensorStatus();
                    colorSensorDetected = false;
                }
                checkColorSensor = false;
            } else if (!(FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE  || FSMManager.robotState == RobotState.READY_TO_GRAB_SPECIMEN) && !checkColorSensor){
                checkColorSensor = true;
            }
            //state models for preset positions
            FSMManager.execute();
            //update drivetrain
            driveTrain.Update();

            //telemetry
            telemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            telemetry.addData("Elbow Target Angle", arm.getElbowTargetPositionInDegrees());
            telemetry.addData("Ave Frequency", freqCounter.getAveFrequency());
            telemetry.addData("Robot State", FSMManager.robotState);
            telemetry.addData("Specimen Pickup State", GrabSpecimenStateTransition.intakeTransitionStep);
            telemetry.addData("Distance", color.getDistance());
            telemetry.addData("Pitch Angle", wrist.getPitchAngle());
            telemetry.addData("Slide Pos", arm.getSlideExtension());
            telemetry.addData("Turn Off Auto Grab", driverControls.turnOffAutoGrab());
            telemetry.update();


            //logging
            if (enableLogging){
                logDriveTrain();
                logArm();
                logEndEffector();
                logStateModels();
                logButtonPressed();
            }
        }
        writePositionsToFile();
    }

    private void initializeGamePads() {
        driverControls = new DriverControls(gamepad1, gamepad2, 1);
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

        GoBildaPinpointDriverRR pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        imu = new IMUforPinpoint(pinpoint);

        localization = new Localization(pinpoint, imu);
        ActionRunner runner = new ActionRunner();
        PinpointDrive drive = new PinpointDrive(hardwareMap,pinpoint.getPositionRR());
        driveTrain = new DriveTrainAuto(FL, FR, BL, BR, imu, runner, drive);
    }
    private void initializeArmAndHome(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slideSwitch = hardwareMap.get(RevTouchSensor.class, "slide switch");
        elbowSwitch = hardwareMap.get(RevTouchSensor.class, "elbow switch");

        debounceTimer = new ElapsedTime();
        debounceTimer.reset();

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setTargetPositionTolerance(10);
        rightSlide.setTargetPositionTolerance(10);
        //pivot.setTargetPositionTolerance(0);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide slideControl = new Slide(leftSlide, rightSlide, slideSwitch);
        Elbow elbow = new Elbow(pivot, elbowSwitch, 100);
        arm = new Arm(slideControl, elbow);


    }
    private void initializeIntake(){
        leftRoller = hardwareMap.get(CRServoImplEx.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServoImplEx.class, "rightRoller");
        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        intake = new BigWheelIntake(leftRoller,rightRoller);
        color = new ColorSensor(colorSensor);

    }
    private void initializePitch(){
        pitch = hardwareMap.get(Servo.class, "pitchLeft");
        wrist = new Wrist(pitch);
    }
    private void initializeEndEffector(){
        initializePitch();
        initializeIntake();
        endEffector = new EndEffectorV2(wrist, claw);
    }
    private void initializeLinearActuator(){
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        RevTouchSensor actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");

        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);
    }
    private void initializeStateModels(){
        FSMManager.initialize(wrist, intake, arm, driveTrain, driverControls,color, linearActuator, alliance, colorSensorDetected);
    }
    private void initializeLED(){
        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        led = new LED(LED);
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
        //LoggerUtil.debug("stateModels", StateModelsZapdos.getDebugString());
    }
    private void logButtonPressed(){
        LoggerUtil.debug("buttonPresses", String.valueOf(driverControls.slideMovement()));
    }
    private void updateLED(){
        if (driveTrain.getLockDriveTrain()) {
            led.setColor(ILED.LEDColor.GREEN);
        } else if (!colorSensorDetected){
            led.setColor(ILED.LEDColor.ORANGE);
        } else if (FSMManager.isAtStart()) {
            led.setColor(ILED.LEDColor.YELLOW);
        } else if (matchTimer.seconds() > 55 && matchTimer.seconds() < 100){
            led.setColor(ILED.LEDColor.WHITE);
        } else if (matchTimer.seconds() > 100 && matchTimer.seconds() < 120){
            led.setColor(ILED.LEDColor.RED);
        } else if (led.isOn()){
            led.turnOff();
        }
    }
    private void writePositionsToFile(){
        try{
            BufferedWriter writer = new BufferedWriter(new FileWriter("/sdcard/Download/teleop/EndTeleOpPresetPositions.csv",false));
            writer.write("");
            writer.flush();
            writer.write("ENTER SUBMERSIBLE PITCH VALUE(1ST NUMBER IN FILE): " + StateModelParameters.EnterSubmersibleStateParameters.pitch);
            writer.newLine();
            writer.write("DEPOSIT POSITION SLIDE LENGTH(2ND TO LAST NUMBER IN FILE): " + StateModelParameters.DepositStateParameters.slideLength);
            writer.newLine();
            writer.write("DEPOSIT POSITION PITCH ANGLE(2ND NUMBER IN FILE): " + StateModelParameters.DepositStateParameters.pitch);
            writer.newLine();
            writer.write("CLIP ELBOW ANGLE:(3RD NUMBER IN FILE): "+ StateModelParameters.PickupSpecimensStateParameters.elbowAngle);
            writer.close();
        } catch (Exception e){
            telemetry.addLine("Failed to write value updates");
            telemetry.update();
        }
    }
}
