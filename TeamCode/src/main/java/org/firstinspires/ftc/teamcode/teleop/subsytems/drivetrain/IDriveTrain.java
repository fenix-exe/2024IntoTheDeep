package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;

import java.util.HashMap;

public interface IDriveTrain {
    public enum DriveType {ROBOT_CENTRIC,FIELD_CENTRIC}
    public void Move(DriveType driveType, double forwardDrive, double strafeDrive, double heading);
    public void setMaxPower(double maxPower);
    public void Follow(PathChain path);
    public void Update();
    public void resetIMU();
    public HashMap getDebugInfo();
    public Pose getCurrentPose();
}
