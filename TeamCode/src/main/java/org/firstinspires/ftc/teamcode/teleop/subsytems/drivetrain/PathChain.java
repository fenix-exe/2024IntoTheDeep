package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.auto.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive;

import java.util.ArrayList;


public class PathChain {
    ArrayList<Pose2d> pathChain;
    public PathChain(ArrayList<Pose2d> pathChain){
        this.pathChain = pathChain;
    }
    public SequentialAction getPathToFollow(PinpointDrive drive){
        ArrayList<MecanumDrive.PIDDrive> action = new ArrayList();
        for (int i = 0; i < pathChain.size(); i++){
            action.add(drive.pidToPointAction(pathChain.get(i)));
        }
        return new SequentialAction();
    }
}
