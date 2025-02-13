package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public abstract class BasePath {
    Pose currentPose;
    public static ArrayList<Point> controlPoints  = new ArrayList<>();
    public static Point endPoint;
    public static String interpolationType;
    public static double interpolationParam1;
    public static double interpolationParam2;
    public PathBuilder builder = new PathBuilder();
    public BasePath(Pose currentPose){
        this.currentPose = currentPose;
    }
    public abstract PathChain getPathChain();
}
