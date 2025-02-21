package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.LoggerUtil;

import java.util.HashMap;

public class DriveTrainWithPedroPathing implements IDriveTrain{
    public Follower follower;
    private boolean manualDrive = true;
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;
    double maxPower=1;


    public DriveTrainWithPedroPathing(HardwareMap hardwareMap, Pose startPose){
        Constants.setConstants(FConstants.class, LConstants.class);
        FL = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        BL = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        BR = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        FR = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        follower = new Follower(hardwareMap);
        resetIMU();
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }
    @Override
    public void Move(DriveType driveType, double forwardDrive, double strafeDrive, double heading) {
        if (!manualDrive && (forwardDrive != 0 || strafeDrive != 0 || heading != 0)){
            follower.startTeleopDrive();
            manualDrive = true;
        }
        follower.setTeleOpMovementVectors(forwardDrive * maxPower, -strafeDrive * maxPower, -heading * maxPower, driveType == DriveType.ROBOT_CENTRIC);
    }

    @Override
    public void setMaxPower(double maxPower) {
        FollowerConstants.maxPower = maxPower;
        this.maxPower = maxPower;
    }

    @Override
    public void Follow(PathChain path) {
        manualDrive = false;
        setMaxPower(RobotConstants.NORMAL_SPEED);
        follower.followPath(path, true);
    }

    @Override
    public void Update() {
        follower.update();
    }

    @Override
    public void resetIMU() {
        try {
            follower.poseUpdater.resetIMU();
        } catch (Exception e){
            LoggerUtil.logException(e);
        }

    }

    @Override
    public HashMap getDebugInfo() {
        HashMap debugInfo = new HashMap<>();
        Pose current_pose = follower.getPose();
        debugInfo.put("X", current_pose.getX());
        debugInfo.put("Y", current_pose.getY());
        debugInfo.put("IMU Yaw", String.valueOf(follower.getPose().getHeading()));
        debugInfo.put("FL Power", String.valueOf(FL.getPower()));
        debugInfo.put("BL Power", String.valueOf(BR.getPower()));
        debugInfo.put("FR Power", String.valueOf(FR.getPower()));
        debugInfo.put("BR Power", String.valueOf(BR.getPower()));
        debugInfo.put("FL Current", String.valueOf(FL.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("BL Current", String.valueOf(BL.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("FR Current", String.valueOf(FR.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("BR Current", String.valueOf(BR.getCurrent(CurrentUnit.MILLIAMPS)));
        return debugInfo;
    }

    @Override
    public Pose getCurrentPose() {
        return follower.getPose();
    }

    @Override
    public void stopFollowing() {
        if (!manualDrive){
            follower.startTeleopDrive();
            manualDrive=true;
        }
    }

    @Override
    public boolean isFollowingPath() {
        return follower.isBusy();
    }
    public boolean atPosition(PathChain pathChain){
        return follower.getPose().roughlyEquals(new Pose(12.6796, 129.6978));
    }
}
