package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class driveCode {
    Gamepad gamepad1;
    Gamepad gamepad1previous;
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;
    IMU imu_IMU;
    double speedMultiplier = 1;
    Telemetry telemetry;
    public driveCode(Gamepad gamepad1, Gamepad gamepad1previous, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IMU imu, Telemetry telemetry){
        this.gamepad1=gamepad1;
        this.gamepad1previous=gamepad1previous;
        this.FL=FL;
        this.FR=FR;
        this.BL=BL;
        this.BR=BR;
        this.imu_IMU = imu;
        this.telemetry = telemetry;
    }
    private double speedMultiplication(){
        if (gamepad1.right_bumper & !gamepad1previous.right_bumper){
            if (speedMultiplier == 1){
                speedMultiplier = 0.5;
            } else {
                speedMultiplier = 1;
            }
        }
        return speedMultiplier;
    }
    public void RobotCentric_Drive() {
        float drive;
        double strafe;
        float yaw;

        drive = gamepad1.left_stick_y * -1;
        strafe = gamepad1.left_stick_x * 1.1;
        yaw = gamepad1.right_stick_x;
        double denominator = Math.max(1, Math.abs(drive+strafe+yaw));
        FL.setPower(((drive + strafe + yaw) / denominator) * speedMultiplication());
        BL.setPower((((drive - strafe) + yaw) / denominator) * speedMultiplication());
        FR.setPower((((drive - strafe) - yaw) / denominator) * speedMultiplication());
        BR.setPower((((drive + strafe) - yaw) / denominator) * speedMultiplication());
    }
    public void FieldCentricDrive() {
        double botHeading;
        float y;
        float x;
        float rx;
        double rotY;
        double rotX;
        double fielddenom;

        botHeading = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        y = gamepad1.left_stick_y * -1;
        x = gamepad1.left_stick_x * 1;
        rx = gamepad1.right_stick_x * 1;
        telemetry.addData("BotH", botHeading);
        if (gamepad1.dpad_down) {
            imu_IMU.resetYaw();
        }
        rotX = 1.1 * (x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI));
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) - y * Math.cos(-botHeading / 180 * Math.PI);
        fielddenom = Math.max(1, Math.abs(rotX+rotY));
        FL.setPower(((rotY + rotX + rx) / fielddenom) * speedMultiplication());
        BL.setPower((((rotY - rotX) + rx) / fielddenom) * speedMultiplication());
        FR.setPower((((rotY - rotX) - rx) / fielddenom) * speedMultiplication());
        BR.setPower((((rotY + rotX) - rx) / fielddenom) * speedMultiplication());
    }
}
