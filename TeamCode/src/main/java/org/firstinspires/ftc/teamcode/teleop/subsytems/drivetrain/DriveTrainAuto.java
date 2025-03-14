package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;

import java.nio.file.Path;
import java.util.HashMap;

import page.j5155.expressway.ftc.actions.ActionRunner;

public class DriveTrainAuto implements IDriveTrain{

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;
    IIMU imu_IMU;
    public double speedMultiplier = 1;
    public ActionRunner runner;
    public PinpointDrive drive;

    public IDriveTrain.DriveType driveType = IDriveTrain.DriveType.FIELD_CENTRIC;  // Robot-Centric = 0, Field-Centric = 1
    private boolean lockDriveTrain;
    public DriveTrainAuto(DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IIMU imu, ActionRunner runner, PinpointDrive drive){
        this.FL=FL;
        this.FR=FR;
        this.BL=BL;
        this.BR=BR;
        this.imu_IMU = imu;
        this.runner = runner;
        this.drive = drive;
        lockDriveTrain = false;
    }


    public void Move(double forwardDrive, double strafeDrive, double headingDrive) {
        if(forwardDrive != 0 || strafeDrive != 0 || headingDrive != 0){
            if (isFollowingPath()){
                stopFollowing();
            }
            if(driveType == IDriveTrain.DriveType.ROBOT_CENTRIC){
                RobotCentric_Drive(forwardDrive, strafeDrive, headingDrive, speedMultiplier);
            } else {
                FieldCentricDrive(forwardDrive, strafeDrive, headingDrive, speedMultiplier);
            }
        } else if (!isFollowingPath()){
            stopDriveTrain();
        }
    }

    public void setMaxPower(double maxPower) {
        speedMultiplier = maxPower;
    }



    public void Update() {
        runner.updateAsync();

    }
    public void Follow(Pose2d pose) {
        if (!lockDriveTrain){
            runner.runAsync(drive.pidToPointAction(pose));
        }
    }
    public void Follow(PathChain path){
        if (!lockDriveTrain){
            runner.runAsync(drive.pidToPointAction(path.getPathToFollow()));
        }
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

    @Override
    public void lockDriveTrain(boolean lock) {
        lockDriveTrain = lock;
        if (lock){
            stopFollowing();
        }
    }
    @Override
    public boolean getLockDriveTrain() {
        return lockDriveTrain;
    }
    public void RobotCentric_Drive(double forwardDrive, double strafeDrive, double turnDrive,double requestedSpeedMultiplier) {
        double allowedSpeedMultiplier = requestedSpeedMultiplier;
        double drive;
        double strafe;
        double yaw;

        drive = forwardDrive;
        strafe = strafeDrive;
        yaw = turnDrive;

        double denominator = Math.max(1, Math.abs(drive+strafe+yaw));
        FL.setPower(((drive + strafe + yaw) / denominator) * allowedSpeedMultiplier);
        BL.setPower((((drive - strafe) + yaw) / denominator) * allowedSpeedMultiplier);
        FR.setPower((((drive - strafe) - yaw) / denominator) * allowedSpeedMultiplier);
        BR.setPower((((drive + strafe) - yaw) / denominator) * allowedSpeedMultiplier);
    }
    public void FieldCentricDrive(double forwardDrive, double strafeDrive, double turnDrive, double requestedSpeedMultiplier) {
        double allowedSpeedMultiplier = requestedSpeedMultiplier;
        double botHeading;
        double y;
        double x;
        double rx;
        double rotY;
        double rotX;
        double fielddenom;

        botHeading = imu_IMU.getYaw();

        y = forwardDrive;
        x = strafeDrive;
        rx = turnDrive;

        rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        fielddenom = Math.max(1, Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx));
        FL.setPower(((rotY + rotX + rx) / fielddenom) * allowedSpeedMultiplier);
        BL.setPower(((rotY - rotX + rx) / fielddenom) * allowedSpeedMultiplier);
        FR.setPower(((rotY - rotX - rx) / fielddenom) * allowedSpeedMultiplier);
        BR.setPower(((rotY + rotX - rx) / fielddenom) * allowedSpeedMultiplier);
    }
    public void stopDriveTrain(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    @Override
    public void setDriveType(IDriveTrain.DriveType driveType) {
        this.driveType = driveType;
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
