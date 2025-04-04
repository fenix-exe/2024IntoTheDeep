package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.stateModels.PresetConfigUtil;
import org.firstinspires.ftc.teamcode.teleop.stateModels.ResetSlideEncoderStateModel;
import org.firstinspires.ftc.teamcode.teleop.stateModels.FSMManager;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforPinpoint;
import org.firstinspires.ftc.teamcode.teleop.subsytems.LED.ILED;
import org.firstinspires.ftc.teamcode.teleop.subsytems.LED.LED;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
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
import java.util.List;

@Config
@TeleOp
public class DebugTeleOp extends LinearOpMode {
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
    LED led;
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
    public static boolean enableLogging=false;
    boolean colorSensorDetected;
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
            telemetry.addLine("NOT DETECTING COLOR SENSOR");
        } else {
            telemetry.addLine("DETECTING COLOR SENSOR");
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
            /*if (driverControls.driveTypeSwitch()){
                if (DriveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
                    DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
                } else{
                    DriveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
                }
            }

            //imu reset
            if (driverControls.resetIMU()){
                driveTrain.resetIMU();
            }

            //speed adjustments
            if (driverControls.microDriveAdjustments()){
                speedMultiplier = RobotConstants.SLOW_SPEED;
            } else {
                speedMultiplier = RobotConstants.NORMAL_SPEED;
            }

            //manual move of the drivetrain
            switch (DriveTrain.driveType) {
                case ROBOT_CENTRIC:
                    driveTrain.RobotCentric_Drive(speedMultiplier);
                    break;
                case FIELD_CENTRIC:
                    driveTrain.FieldCentricDrive(speedMultiplier);
                    break;
            }

            //manual control for slide
            if (Math.abs(driverControls.slideMovement()) > 0){
                arm.moveSlide(driverControls.slideMovement(), driverControls.removeArmRules());
            } else if (driverControls.slideStopped()){
                //prevents slides from moving after the drivers let go of the joystick
                arm.holdSlide();
            }

            //manual control for elbow
            if (Math.abs(driverControls.pivotJoystick()) > 0){
                arm.moveElbow(driverControls.pivotJoystick());
            } else if (driverControls.pivotManualStopped()){
                //prevents elbow from moving after the drivers let go of the joystick
                arm.holdElbow();
            }

            //Manual control for wrist up
            //precedence is the following - continous diff > 42in > manual diff
            if (driverControls.continuousDiffUp()){
                wrist.manualControlPitch(1);
            } else if (arm.getSlideExtension() > arm.getMaximumSlideExtensionAllowedInInches() - 7
                    && arm.getElbowAngleInDegrees() < 10){
                // When slide is extended, making sure pitch is down or we can break the 42in limit
                wrist.presetPositionPitch(-90);
            } else if (driverControls.diffUp()){
                wrist.manualControlPitch(15);
            }

            //manual control for wrist down
            if (driverControls.continuousDiffDown()){
                wrist.manualControlPitch(-1);
            } else if (driverControls.diffDown()){
                wrist.manualControlPitch(-15);
            }

            if (driverControls.diffLeft()){
                wrist.manualControlRoll(-45);
            }
            if (driverControls.diffRight()){
                wrist.manualControlRoll(45);
            }

            //manual control for claw
            if (driverControls.openClaw()){
                claw.openClaw();
            }
            if (driverControls.closeClaw()){
                claw.closeClaw();
            }

            //resetting encoders on gamepad press
            if (driverControls.resetEncoders()){
                arm.resetEncoders();
            }*/

            //homing
            /*if (driverControls.homeArm()){
                home();
            }*/

            //run touch sensor fsm for resetting slides
           /* ResetSlideEncoderStateModel.execute();

            //linear actuator code for driver control outside of state models
            if (driverControls.linearActuatorUp()){
                if (driverControls.microDriveAdjustments()){
                    //for manual movements
                    double pos = linearActuator.getLinearActuatorPositionInches() + 1;
                    linearActuator.goToTargetPositionInches(pos);
                    //preset positions
                    linearActuator.goToTargetPositionInches(9.5);
                }
            }
            if (driverControls.linearActuatorDown()){
                if (driverControls.microDriveAdjustments()){
                    //for manual movements
                    if (!linearActuator.getLimitSwitchState()){
                        //prevents the linear actuator from driving into the ground
                        linearActuator.goToTargetPositionInches(Math.max(linearActuator.getLinearActuatorPositionInches() - 0.25,0.5));
                    }
                } else {
                    //preset position
                    linearActuator.goToTargetPositionInches(5.75);
                }

            }

            //led blinking
            updateLED();

            if (driverControls.escapePresets()){
                arm.holdArm();;
                driveTrain.lockDriveTrain(false);
                FSMManager.stopTransitions();
                FSMManager.setRobotStateToStart();
            }
            //state models for preset positions
            FSMManager.execute();

            //telemetry
            /*multiTelemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            multiTelemetry.addData("Target Pos Linear Actuator", linearActuatorMotor.getTargetPosition());
            multiTelemetry.addData("Elbow Current", pivot.getCurrent(CurrentUnit.MILLIAMPS));
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
            telemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            telemetry.addData("Ave Frequency", freqCounter.getAveFrequency());
            telemetry.addData("Color Sensor Distance", color.getDistance());
            multiTelemetry.update();

            //logging
            /*if (enableLogging){
                logDriveTrain();
                logArm();
                logEndEffector();
                logStateModels();
                logButtonPressed();
            }*/
        }
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
        /*pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2d(0,0,0));*/
        imu = new IMUforPinpoint(pinpoint);

        localization = new Localization(pinpoint, imu);

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, imu);
    }
    private void initializeArmAndHome(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        debounceTimer = new ElapsedTime();
        debounceTimer.reset();

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setTargetPositionTolerance(0);
        rightSlide.setTargetPositionTolerance(0);
        pivot.setTargetPositionTolerance(0);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        wrist = new Wrist(pitch);
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
    }
    private void initializeStateModels(){
        FSMManager.initialize(wrist, null, arm, driveTrain, driverControls,color, linearActuator, null, colorSensorDetected);
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
    private void home(){
        //homing the slide
        while (!arm.isSlideTouchSensorPressed() && !isStopRequested()){
            arm.setSlidePower(-0.2);
            telemetry.addData("slide switch state", arm.isSlideTouchSensorPressed());
            telemetry.addData("Elbow Angle", arm.getElbowAngleInDegrees());
            telemetry.update();
        }
        arm.setSlidePower(0);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Homing the elbow
        while (!arm.detectingMagneticLimitSwitch() && !isStopRequested()){
            arm.setElbowPower(-0.2);
        }
        while (arm.detectingMagneticLimitSwitch() && !isStopRequested()){
            arm.setElbowPower(-0.4);
        }
        while (!arm.detectingMagneticLimitSwitch() && !isStopRequested()){
            arm.setElbowPower(0.4);
        }
        arm.setElbowPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot.setTargetPosition(-266);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(1);

        while((Math.abs(pivot.getCurrentPosition() - pivot.getTargetPosition()) > 12)){

        }

        pivot.setPower(0);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !isStopRequested()){
            telemetry.addLine("ELBOW IS HOMED");
            telemetry.update();
            linearActuator.setLinearActuatorPower(-0.5);
        }
        linearActuator.setLinearActuatorPower(0);

        linearActuator.resetEncoders();
    }
    private void updateLED(){
        if (driveTrain.getLockDriveTrain()){
            led.setColor(ILED.LEDColor.GREEN);
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
}
