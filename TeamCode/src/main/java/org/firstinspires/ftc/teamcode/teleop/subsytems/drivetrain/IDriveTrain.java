package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.HashMap;

public interface IDriveTrain {
    public enum DriveType {ROBOT_CENTRIC,FIELD_CENTRIC}
    public void Move(double forwardDrive, double strafeDrive, double heading);
    public void setMaxPower(double maxPower);
    public void Update();
    public void resetIMU();
    public HashMap getDebugInfo();
    public Pose2d getCurrentPose();
    public void stopFollowing();
    public boolean isFollowingPath();
    public boolean getLockDriveTrain();
    public void stopDriveTrain();
    public void setDriveType(DriveType driveType);
    public void Follow(Pose2d pose);
}
