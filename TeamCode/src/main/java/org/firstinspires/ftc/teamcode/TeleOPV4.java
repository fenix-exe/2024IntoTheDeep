package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
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

import org.firstinspires.ftc.teamcode.robot.ConfigUtil;
import org.firstinspires.ftc.teamcode.robot.RobotActions;
import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsytems.differential.Differential;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.ActiveIntake;
import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffector;
import org.firstinspires.ftc.teamcode.util.FrequencyCounter;
import org.firstinspires.ftc.teamcode.util.LoggerUtil;

import java.util.HashMap;
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
    Differential diffCode;
    IMU imu;
    RevColorSensorV3 activeIntakeSensor;
    RevTouchSensor limitSwitch;
    FrequencyCounter freqCounter;
    int PHYSICALMAXEXTENSION = 2500;

    private static boolean isTelemetryEnabled = true;
    private static boolean isLoggingEnabled = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        ConfigUtil.loadPresetsFromConfig();

        RobotActions actions = new RobotActions();
        RobotCore.initialize(driverControls, driveTrain, arm, endEffector);
        waitForStart();
        diffCode.setDifferentialPosition(-90,-90);
        freqCounter = new FrequencyCounter();


        while (opModeIsActive()){
            freqCounter.count();
            driverControls.update();

            Set directive = driverControls.getUserIntents();
            boolean directiveContainsChangePreset = RobotCore.updatePresetPositions(actions, directive);
            RobotCore.updateRobotActionsforArm(actions, directive);
            RobotCore.updateRobotActionsForEndEffector(actions, directive);
            RobotCore.updateRobotActionsforDriveTrain(actions, directive);

            logDebugInfo(directive, actions);

            //execution of actions
            actions.execute();
            actions.removeCompleteAndCancelled();

            if (directiveContainsChangePreset) {
                ConfigUtil.writePresetsToConfig();
            }
        }
        LoggerUtil.logFlush();

    }


    private void logDebugInfo(Set directive, RobotActions actions) {
        if (isTelemetryEnabled || isLoggingEnabled) {
            String directives = directive.toString();
            String actions_list = actions.toString();
            HashMap arm_debugInfo = arm.getDebugInfo();
            HashMap endEffector_debugInfo = endEffector.getDebugInfo();
            HashMap driveTrain_debugInfo = driveTrain.getDebugInfo();
            updateTelemetry(directive, actions_list, arm_debugInfo, endEffector_debugInfo, driveTrain_debugInfo);
            updateLogging(directives, actions_list, arm_debugInfo, endEffector_debugInfo, driveTrain_debugInfo);
        }
    }

    private static void updateLogging(String directives, String actions_list, HashMap arm_debugInfo, HashMap endEffector_debugInfo, HashMap driveTrain_debugInfo) {
        LoggerUtil.debug("Directive: " + directives);
        LoggerUtil.debug("Actions: " + actions_list);
        LoggerUtil.debug("Slide Extension: " + arm_debugInfo.get("Slide Extension"));
        LoggerUtil.debug("Slide Limit: " + arm_debugInfo.get("Slide Limit"));
        LoggerUtil.debug("Slide Current: " + arm_debugInfo.get("Slide Current"));
        LoggerUtil.debug("Elbow Angle: " + arm_debugInfo.get("Elbow Angle"));
        LoggerUtil.debug("Elbow Current: " + arm_debugInfo.get("Elbow Current"));
        LoggerUtil.debug("Differential Pitch: " + endEffector_debugInfo.get("Differential Pitch"));
        LoggerUtil.debug("Differential Roll: " + endEffector_debugInfo.get("Differential Roll"));
        LoggerUtil.debug("Active Intake Power: " + endEffector_debugInfo.get("Active Intake Power"));
        LoggerUtil.debug("DriveTrain Motor Powers: " + driveTrain_debugInfo.get("FL Power") + ", " + driveTrain_debugInfo.get("FR Power") + ", " + driveTrain_debugInfo.get("BL Power") + ", " + driveTrain_debugInfo.get("BR Power"));
        LoggerUtil.debug("DriveTrain Motor Currents: " + driveTrain_debugInfo.get("FL Current") + ", " + driveTrain_debugInfo.get("FR Current") + ", " + driveTrain_debugInfo.get("BL Current") + ", " + driveTrain_debugInfo.get("BR Current"));
        LoggerUtil.debug("IMU Yaw: " + driveTrain_debugInfo.get("IMU Yaw"));
        LoggerUtil.debug("Drive Type: " + driveTrain_debugInfo.get("Drive Type"));
    }

    private void updateTelemetry(Set directive, String actions_list, HashMap arm_debugInfo, HashMap endEffector_debugInfo, HashMap driveTrain_debugInfo) {
        if (isTelemetryEnabled){
            telemetry.addData("Intent", directive);
            telemetry.addData("actions", actions_list);
            telemetry.addData("Slide extension", arm_debugInfo.get("Slide Extension"));
            telemetry.addData("Slide limit", arm_debugInfo.get("Slide Limit"));
            telemetry.addData("Elbow angle", arm_debugInfo.get("Elbow Angle"));
            telemetry.addData("IMU yaw", driveTrain_debugInfo.get("IMU Yaw"));
            telemetry.addData("pitch", endEffector_debugInfo.get("Differential Pitch"));
            telemetry.addData("roll", endEffector_debugInfo.get("Differential Roll"));
            telemetry.addData("slide current", arm_debugInfo.get("Slide Current"));
            telemetry.addData("elbow current", arm_debugInfo.get("Elbow Current"));
            telemetry.addData("active intake power", endEffector_debugInfo.get("Active Intake Power"));
            telemetry.addData("average loop frequency", freqCounter.getAveFrequency());
            //telemetry.addData("Is the hall effect sensor triggered?", limitSwitch.isPressed());
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
        intake = hardwareMap.get(CRServo.class, "intake");
        activeIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "activeIntakeSensor");
        activeIntakeCode = new ActiveIntake(intake, activeIntakeSensor);
    }
    private void initializeDifferential(){
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));
        diffCode = new Differential(left,right);
    }
    private void initializeEndEffector(){
        initializeDifferential();
        initializeIntake();
        endEffector = new EndEffector(activeIntakeCode, diffCode);
    }

}
