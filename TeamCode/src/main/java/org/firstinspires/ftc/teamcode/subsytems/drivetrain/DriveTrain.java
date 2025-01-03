package org.firstinspires.ftc.teamcode.subsytems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    public enum DriveType {ROBOT_CENTRIC,FIELD_CENTRIC}
    Gamepad gamepad1;
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;
    IMU imu_IMU;
    double speedMultiplier = 1;

    public static DriveType driveType = DriveType.FIELD_CENTRIC;  // Robot-Centric = 0, Field-Centric = 1

    Telemetry telemetry;
    public DriveTrain(Gamepad gamepad1, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IMU imu, Telemetry telemetry){
        this.gamepad1=gamepad1;
        this.FL=FL;
        this.FR=FR;
        this.BL=BL;
        this.BR=BR;
        this.imu_IMU = imu;
        this.telemetry = telemetry;
    }

    public void RobotCentric_Drive() {
        float drive;
        double strafe;
        float yaw;

        drive = gamepad1.left_stick_y * -1;
        strafe = gamepad1.left_stick_x * 1.1;
        yaw = gamepad1.right_stick_x;
        double denominator = Math.max(1, Math.abs(drive+strafe+yaw));
        FL.setPower(((drive + strafe + yaw) / denominator) * speedMultiplier);
        BL.setPower((((drive - strafe) + yaw) / denominator) * speedMultiplier);
        FR.setPower((((drive - strafe) - yaw) / denominator) * speedMultiplier);
        BR.setPower((((drive + strafe) - yaw) / denominator) * speedMultiplier);
    }
    public void RobotCentric_Drive(double speedMultiplier) {
        float drive;
        double strafe;
        float yaw;

        drive = gamepad1.left_stick_y * -1;
        strafe = gamepad1.left_stick_x * 1.1;
        yaw = gamepad1.right_stick_x;
        double denominator = Math.max(1, Math.abs(drive+strafe+yaw));
        FL.setPower(((drive + strafe + yaw) / denominator) * speedMultiplier);
        BL.setPower((((drive - strafe) + yaw) / denominator) * speedMultiplier);
        FR.setPower((((drive - strafe) - yaw) / denominator) * speedMultiplier);
        BR.setPower((((drive + strafe) - yaw) / denominator) * speedMultiplier);
    }
    public void FieldCentricDrive() {
        double botHeading;
        double y;
        double x;
        double rx;
        double rotY;
        double rotX;
        double fielddenom;



        botHeading = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1;
        rx = gamepad1.right_stick_x * 1;
        telemetry.addData("BotH", botHeading);

        rotX = 1.1 * (x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI));
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        fielddenom = Math.max(1, Math.abs(rotX+rotY));
        FL.setPower(((rotY + rotX + rx) / fielddenom) * speedMultiplier);
        BL.setPower((((rotY - rotX) + rx) / fielddenom) * speedMultiplier);
        FR.setPower((((rotY - rotX) - rx) / fielddenom) * speedMultiplier);
        BR.setPower((((rotY + rotX) - rx) / fielddenom) * speedMultiplier);
    }
    public void FieldCentricDrive(double speedMultiplier) {
        double botHeading;
        double y;
        double x;
        double rx;
        double rotY;
        double rotX;
        double fielddenom;



        botHeading = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1;
        rx = gamepad1.right_stick_x * 1;
        telemetry.addData("BotH", botHeading);

        rotX = 1.1 * (x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI));
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        fielddenom = Math.max(1, Math.abs(rotX+rotY));
        FL.setPower(((rotY + rotX + rx) / fielddenom) * speedMultiplier);
        BL.setPower((((rotY - rotX) + rx) / fielddenom) * speedMultiplier);
        FR.setPower((((rotY - rotX) - rx) / fielddenom) * speedMultiplier);
        BR.setPower((((rotY + rotX) - rx) / fielddenom) * speedMultiplier);
    }
    public void resetIMU(){
        imu_IMU.resetYaw();
    }
}
