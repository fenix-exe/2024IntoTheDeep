package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;

import java.util.HashMap;

import page.j5155.expressway.ftc.actions.ActionRunner;

public class DriveTrainAuto{

    public enum DriveType {ROBOT_CENTRIC,FIELD_CENTRIC}
    Gamepad gamepad1;
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;
    IIMU imu_IMU;
    public double speedMultiplier = 1;
    public ActionRunner runner;
    public PinpointDrive drive;

    public static DriveType driveType = DriveType.FIELD_CENTRIC;  // Robot-Centric = 0, Field-Centric = 1
    public rrDrive(Gamepad gamepad1, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IIMU imu, ActionRunner runner, Telemetry telemetry, PinpointDrive drive){
        this.gamepad1=gamepad1;
        this.FL=FL;
        this.FR=FR;
        this.BL=BL;
        this.BR=BR;
        this.imu_IMU = imu;
        this.runner = runner;
        this.drive = drive;
    }


    public void Move(IDriveTrain.DriveType driveType, double forwardDrive, double strafeDrive, double heading) {
        if(driveType == IDriveTrain.DriveType.ROBOT_CENTRIC){
            RobotCentric_Drive(speedMultiplier);
        } else {
            FieldCentricDrive(speedMultiplier);
        }
    }

    public void setMaxPower(double maxPower) {
        speedMultiplier = maxPower;
    }



    public void Update() {
        runner.updateAsync();

    }
    public void Follow(Pose2d pose) {

        runner.runAsync(drive.pidToPointAction(pose));
    }

    public void stopFollowing() {
        runner.getRunningActions().clear();

    }

    public Pose2d getCurrentPose() {
        return drive.getPoseEstimate();
    }

    public boolean isFollowingPath() {
        return !runner.getRunningActions().isEmpty();
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
        if (gamepad1.right_stick_x < -0.5  || gamepad1.right_stick_x > 0.5){
            yaw = gamepad1.right_stick_x;
        } else {
            yaw = 0;
        }
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



        botHeading = imu_IMU.getYaw();
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1;
        if (gamepad1.right_stick_x < -0.5 || gamepad1.right_stick_x > 0.5){
            rx = gamepad1.right_stick_x * 1;
        } else {
            rx = 0;
        }

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

        botHeading = imu_IMU.getYaw();

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1;
        rx = gamepad1.right_stick_x * 1;

        rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        fielddenom = Math.max(1, Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx));
        FL.setPower(((rotY + rotX + rx) / fielddenom) * speedMultiplier);
        BL.setPower(((rotY - rotX + rx) / fielddenom) * speedMultiplier);
        FR.setPower(((rotY - rotX - rx) / fielddenom) * speedMultiplier);
        BR.setPower(((rotY + rotX - rx) / fielddenom) * speedMultiplier);
    }
    public void stopDriveTrain(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    public void resetIMU(){
        imu_IMU.resetYaw();
    }
    public HashMap getDebugInfo() {
        /*telemetry.addData("Slide extension", arm.getSlideExtension());
        telemetry.addData("Slide target position", arm.getSlideExtension());
        telemetry.addData("Slide limit", arm.getSlideMaxLengthIn42Inches(arm.getElbowAngleInTicks()));
        telemetry.addData("Elbow angle", arm.getElbowAngleInDegrees());
        telemetry.addData("Elbow target position", pivot.getTargetPosition());*/

        HashMap debugInfo = new HashMap<>();
        debugInfo.put("X",0);
        debugInfo.put("Y",0);
        debugInfo.put("IMU Yaw", String.valueOf(imu_IMU.getYaw()));
        debugInfo.put("FL Power", String.valueOf(FL.getPower()));
        debugInfo.put("BL Power", String.valueOf(BL.getPower()));
        debugInfo.put("FR Power", String.valueOf(FR.getPower()));
        debugInfo.put("BR Power", String.valueOf(BR.getPower()));
        debugInfo.put("FL Current", String.valueOf(FL.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("BL Current", String.valueOf(BL.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("FR Current", String.valueOf(FR.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("BR Current", String.valueOf(BR.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("Drive Type", String.valueOf(driveType));
        return debugInfo;
    }




}
