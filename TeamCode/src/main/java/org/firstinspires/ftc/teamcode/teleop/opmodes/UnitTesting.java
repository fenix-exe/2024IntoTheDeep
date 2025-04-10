package org.firstinspires.ftc.teamcode.teleop.opmodes;

import static org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforREV;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
@TeleOp(group="Testing")
public class UnitTesting extends LinearOpMode {

     enum mode {DRIVETRAIN, ELBOW, SLIDES, INTAKE;}public static double x = 39.7, y = 65, heading = -180;public GoBildaPinpointDriverRR pinpoint;Servo pitchLeft, pitchRight;Wrist wrist;DcMotorEx elbowMotor;Elbow elbow;ColorSensor colorSensor;RevColorSensorV3 hardwareColorSensor;CRServoImplEx leftRoller, rightRoller;BigWheelIntake intake;enum Alliance{RED,BLUE}UnitTesting.Alliance alliance = UnitTesting.Alliance.RED;DriveTrain driveTrain;IMU imu;DriverControls driverControls;double speedMultiplier, intakePower = 0;private DcMotorEx leftslide, rightslide;boolean exitingWrongColor = false, detectingColor = false;

    @Override
    public void runOpMode() throws InterruptedException {
        mode Mode = mode.DRIVETRAIN;
        telemetry.addData("Instructions", "Press the d-pad to cycle between units to test. In any unit, hold down Gamepad 1's x to see instructions.");
        waitForStart();
        leftslide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftslide.setDirection(DcMotor.Direction.REVERSE);
        pitchLeft = hardwareMap.get(Servo.class, "pitchLeft");
        pitchRight = hardwareMap.get(Servo.class, "pitchRight");
        wrist = new Wrist(pitchLeft);
        hardwareColorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        leftRoller = hardwareMap.get(CRServoImplEx.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServoImplEx.class, "rightRoller");
        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = new ColorSensor(hardwareColorSensor);
        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        RevTouchSensor limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow = new Elbow(elbowMotor, limitSwitch, 90);
        intake = new BigWheelIntake(leftRoller,rightRoller);
        initializeDriveTrain();
        while (opModeIsActive()) {
            telemetry.addData("Current Unit", Mode.toString());
            if (Mode == mode.DRIVETRAIN) {
                if(gamepad1.x) {
                    telemetry.addData("Instructions", "Use regular drive controls. Pinpoint data appears in telemetry.");
                }
                pinpointTesting();
                drivetrainTesting();
            }
            if (Mode == mode.ELBOW) {
                if (gamepad1.x) {
                    telemetry.addData("Instructions", "Sorry, this functionality has not been added yet. Stay tuned for updates :)");
                }
                elbowManualTesting();
            }
            if (Mode == mode.SLIDES) {
                if (gamepad1.x) {
                    telemetry.addData("Instructions", "Use the right stick up and down to set slide power for manual control.");
                }
                slideManualTesting();
            }
            if (Mode == mode.INTAKE) {
                if (gamepad1.x) {
                    telemetry.addData("Instructions", "Press a to rise the wrist. Press b to lower the wrist. Press y to reset the wrist to center. Left and right bumpers change alliance, left and right triggers toggle intake power, and the left stick button cuts off intake.");
                }
                wristTesting();
                colorSensorTesting();
                intakeTesting();

            }
            if (gamepad1.dpad_up) {
                Mode = mode.DRIVETRAIN;
            }
            else if (gamepad1.dpad_down) {
                Mode = mode.ELBOW;
            }
            else if (gamepad1.dpad_left) {
                Mode = mode.SLIDES;
            }
            else if (gamepad1.dpad_right) {
                Mode = mode.INTAKE;
            }
            telemetry.update();
        }
    }
    private void initializePinPoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,PARAMS.pinpointDeviceName);


        //set up ftc dashboard telemetry
        MultipleTelemetry multi = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // reset pinpoint and calibrate
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2d(x,y,Math.toRadians(heading)));
    }
    private void pinpointTesting() {
        pinpoint.update();

        //display pinpoint position on dashboard and ds
        telemetry.addData("pose x", pinpoint.getPositionRR().position.x);
        telemetry.addData("pose y", pinpoint.getPositionRR().position.y);
        telemetry.addData("pose heading", Math.toDegrees(pinpoint.getPositionRR().heading.toDouble()));
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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        //imu.resetYaw();
        IIMU iimu = new IMUforREV(imu);

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, iimu);
    }
    private void wristTesting() {
        if (gamepad1.a){
            wrist.manualControlPitch(0.005);
        }
        if (gamepad1.b){
            wrist.manualControlPitch(-0.005);
        }
        if (gamepad1.y){
            wrist.presetPositionPitch(0.5);
        }
        telemetry.addData("Left Servo Pos", pitchLeft.getPosition());
        telemetry.addData("Right Servo Pos", pitchRight.getPosition());
    }
    private void colorSensorTesting() {
        if (gamepad1.left_bumper) {
            alliance = Alliance.BLUE;
        }
        if (gamepad1.right_bumper) {
            alliance = Alliance.RED;
        }
        if (colorSensor.detectingBlue()){
            if (alliance == UnitTesting.Alliance.BLUE){
                telemetry.addLine("PICKED UP ALLIANCE COLOR, READY TO RETRACT SLIDES");
                intakePower = 0;
            } else {
                telemetry.addLine("EJECT");
                exitingWrongColor = true;
                intakePower = -1;
            }
        } else if (colorSensor.detectingRed()){
            if (alliance == UnitTesting.Alliance.BLUE){
                telemetry.addLine("EJECT");
                exitingWrongColor = true;
                intakePower = -1;
            } else {
                telemetry.addLine("PICKED UP ALLIANCE COLOR, READY TO RETRACT SLIDES");
                intakePower = 0;
            }
        } else if (colorSensor.detectingYellow()){
            telemetry.addLine("PICKED UP YELLOW, READY TO RETRACT SLIDES");
            intakePower = 0;
        }

        if (exitingWrongColor && !detectingColor){
            exitingWrongColor = false;
            intakePower = 0;
        }

    }
    private void intakeTesting() {
        if (gamepad1.left_stick_button){
            intakePower = 0;
        }
        if (gamepad1.left_trigger > 0){
            intakePower = 1;
        }
        if (gamepad1.right_trigger > 0){
            intakePower = -1;
        }

        if (intakePower == 0){
            intake.stop();
        } else if (intakePower == 1){
            intake.intake();
        } else {
            intake.outtake();
        }
        telemetry.addData("H", colorSensor.getH());
        telemetry.addData("Detecting Blue", colorSensor.getBlue());
    }
    private void drivetrainTesting() {
        driverControls.update();

        //driving code
        if (driverControls.driveTypeSwitch()) {
            if (DriveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC) {
                DriveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
            } else {
                DriveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            }

        }

        if (driverControls.resetIMU()) {
            driveTrain.resetIMU();
        }

        //speed adjustments
        if (driverControls.slowMode()) {
            speedMultiplier = RobotConstants.SLOW_SPEED;
        } else {
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
    }
    private void slideManualTesting() {
        if (leftslide.getCurrentPosition() < 0 && -gamepad1.right_stick_y < 0) {
            leftslide.setPower(0);
            rightslide.setPower(0);
        } else if (leftslide.getCurrentPosition() > 3496 && -gamepad1.right_stick_y > 0) {
            leftslide.setPower(0);
            rightslide.setPower(0);
        } else {
            leftslide.setPower(-gamepad1.right_stick_y);
            rightslide.setPower(-gamepad1.right_stick_y);

        }
        telemetry.addData("slide current", leftslide.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("slide encoder", leftslide.getCurrentPosition());
        // Put loop blocks here.
        telemetry.update();
    }
    private void elbowManualTesting() {
        if(gamepad1.a){
            elbow.setTargetAngle(90);
        }
        if(gamepad1.b){
            elbow.setTargetAngle(0);
        }
        if(gamepad1.y){
            elbow.setTargetAngle(45);
        }
        if(gamepad1.left_stick_y > 0.1){
            double currentAngle = elbow.getElbowAngle();
            elbow.setTargetAngle(currentAngle-3);
        }
        if(gamepad1.left_stick_y < -0.1){
            double currentAngle = elbow.getElbowAngle();
            elbow.setTargetAngle(currentAngle+3);
        }
        telemetry.addData("Elbow Position", elbow.getElbowAngle());
    }
}
