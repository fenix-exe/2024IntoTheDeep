package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class MoveXDirection extends BasePath{
    static MoveXDirection instance = null;
    double distance = 0;
    private MoveXDirection(){
        super();
        endPointAsString = new PointsAsStrings("32", "32");
    }
    public static MoveXDirection getInstance(){
        if(instance==null){
            instance = new MoveXDirection();
        }
        return instance;
    }
    public void setDistance(double distance, Pose currentPose){
        this.distance = distance;
        endPointAsString = new PointsAsStrings(String.valueOf(currentPose.getX() + distance), String.valueOf(currentPose.getY()));
    }
    @Override
    public PathChain getPathChain(Pose currentPose) {
        PathBuilder builder = new PathBuilder();
        PathChain path = builder.addPath(
                new Path(
                        new BezierLine(
                                new Point(currentPose),
                                new Point(currentPose.getX() + distance, currentPose.getY(), Point.CARTESIAN)
                        )
                )).setConstantHeadingInterpolation(0).build();
        return path;
    }
}
