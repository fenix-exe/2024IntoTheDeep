package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;

public abstract class BasePath {
    Pose currentPose;
    public PathBuilder builder = new PathBuilder();
    public BasePath(Pose currentPose){
        this.currentPose = currentPose;
    }
    public abstract PathChain getPathChain();
}
