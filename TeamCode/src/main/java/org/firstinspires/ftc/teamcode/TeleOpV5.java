package org.firstinspires.ftc.teamcode;

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

@TeleOp
public class TeleOpV5 extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        initializeArmAndHome();
        initializeEndEffector();
        PresetConfigUtil.loadPresetsFromConfig();
        StateModels.initialize(arm, wrist, driverControls);
        DriveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;


        waitForStart();

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

            switch (DriveTrain.driveType) {
                case ROBOT_CENTRIC:
                    driveTrain.RobotCentric_Drive(1);
                    break;
                case FIELD_CENTRIC:
                    driveTrain.FieldCentricDrive(1);
                    break;
            }

            //manual control for arm
            if (Math.abs(driverControls.slideMovement()) > 0){
                arm.moveSlide(driverControls.slideMovement(), driverControls.removeArmRules());
            }
            if (Math.abs(driverControls.pivotJoystick()) > 0){
                arm.moveElbow(driverControls.pivotJoystick());
            }

            //manual control for wrist
            if (driverControls.diffDown()){
                wrist.manualControlPitch(-1);
                telemetry.addLine("Wrist Down");
            }
            if (driverControls.diffUp()){
                wrist.manualControlPitch(1);
                telemetry.addLine("Wrist Up");
            }
            if (driverControls.diffLeft()){
                wrist.manualControlRoll(-1);
                telemetry.addLine("Wrist Left");
            }
            if (driverControls.diffRight()){
                wrist.manualControlRoll(1);
                telemetry.addLine("Wrist Right");
            }

            //manual control for claw
            if (driverControls.intakenewForward() > 0.01){
                claw.openClaw();
            }
            if (driverControls.intakenewBackward() > 0.01){
                claw.closeClaw();
            }


            //state models for preset positions
            StateModels.presetPositionDriveStateModel(0,-90,58,0);
            StateModels.presetPositionIntakeStateModel(-30,-90,0,2);
            StateModels.presetPositionDepositStateModel(50,-90,70,28.5);


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

        Slide slideControl = new Slide(slide, RobotConstants.PHYSICAL_MAX_EXTENSION);
        Elbow elbow = new Elbow(pivot, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2300);
        arm = new Arm(slideControl, elbow, RobotConstants.PHYSICAL_MAX_EXTENSION);

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




}
