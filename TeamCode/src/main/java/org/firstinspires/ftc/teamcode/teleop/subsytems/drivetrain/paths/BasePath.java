package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public abstract class BasePath {
    Pose currentPose;
    Pose finalDestination;
    public PathBuilder builder = new PathBuilder();
    public BasePath(Pose currentPose, Pose finalDestination){
        this.currentPose = currentPose;
        this.finalDestination = finalDestination;
    }
    public abstract PathChain getPathChain();
    public abstract void lockPath();
    public boolean closeToDestination(){
        return (Math.pow((currentPose.getX() - finalDestination.getX()),2) + Math.pow((currentPose.getY() - finalDestination.getY()),2)) < 144 && (Math.abs(currentPose.getHeading()-finalDestination.getHeading()) < 0.7 || Math.abs(currentPose.getHeading()-2 * Math.PI - finalDestination.getHeading()) < 0.7);
    }
}
