package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake;

public class TELEOP extends LinearOpMode {
    driveCode driverCode;
    activeIntake activeIntakeCode;
    DriverControls controls;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad gamepad1current;
    Gamepad gamepad2current;
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;
    CRServo intake;
    IMU imu;
    double speedMultiplication = 1;
    private enum driveType {FIELD, ROBOT}
    private enum speed {FAST, SLOW}
    driveType drive;
    speed speedMultiplier;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1current = new Gamepad();
        gamepad2current = new Gamepad();

        gamepad1previous = new Gamepad();
        gamepad2previous = new Gamepad();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        gamepad1previous.copy(gamepad1);
        gamepad2previous.copy(gamepad2);
        driverCode = new driveCode(gamepad1, gamepad1previous, FL, BL, FR, BR, imu, telemetry);
        activeIntakeCode = new activeIntake(gamepad2, gamepad2previous, intake);
        controls = new DriverControls(gamepad1current, gamepad2current, gamepad1previous, gamepad2previous);
        drive = driveType.ROBOT;

        waitForStart();

        while (opModeIsActive()){
            gamepad1previous.copy(gamepad1current);
            gamepad2previous.copy(gamepad2current);

            gamepad1current.copy(gamepad1);
            gamepad2current.copy(gamepad2);

            controls = new DriverControls(gamepad1current, gamepad2current, gamepad1previous, gamepad2previous);

            if (controls.driveTypeSwitch()){
                if (drive == driveType.ROBOT){
                    drive = driveType.FIELD;
                } else {
                    drive = driveType.ROBOT;
                }
            }
            if (controls.slowMode()){
                if (speedMultiplier == speed.FAST){
                    speedMultiplier = speed.SLOW;
                } else {
                    speedMultiplier = speed.FAST;
                }
            }



            //switch statements for state machines
            switch (speedMultiplier){
                case SLOW:
                    speedMultiplication = 0.5;
                    break;
                default:
                    speedMultiplication = 1;
                    break;
            }

            switch (drive) {
                case FIELD:
                    driverCode.FieldCentricDrive(speedMultiplication);
                    break;
                default:
                    driverCode.RobotCentric_Drive(speedMultiplication);
                    break;
            }
        }

    }
}
