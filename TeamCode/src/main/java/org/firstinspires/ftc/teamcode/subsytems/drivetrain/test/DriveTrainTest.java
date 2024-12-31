package org.firstinspires.ftc.teamcode.subsytems.drivetrain.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class DriveTrainTest extends LinearOpMode {
    DriveTrain driveTrain;
    IMU imu;
    DriverControls driverControls;
    double speedMultiplier;

    private void initializeGamePads() {
        driverControls = new DriverControls(gamepad1, gamepad2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeGamePads();
        initializeDriveTrain();
        DriveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;


        waitForStart();

        while (opModeIsActive()) {

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
            if (driverControls.microDriveAdjustments()) {
                speedMultiplier = RobotConstants.EXTRA_SLOW;
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

        driveTrain = new DriveTrain(gamepad1, FL, FR, BL, BR, imu, telemetry);
    }
}
