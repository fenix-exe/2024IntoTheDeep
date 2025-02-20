package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;

public class MoveYDirection extends BasePath{
    static MoveYDirection instance = null;
    double distance = 0;
    private MoveYDirection(){
        super();
        endPointAsString = new PointsAsStrings("32","32");
    }
    public static MoveYDirection getInstance(){
        if(instance==null){
            instance = new MoveYDirection();
        }
        return instance;
    }
    public void setDistance(double distance, Pose currentPose){
        this.distance = distance;
        endPointAsString = new PointsAsStrings(String.valueOf(currentPose.getX()), String.valueOf(currentPose.getY() + distance));
    }
    @Override
    public PathChain getPathChain(Pose currentPose) {
        PathBuilder builder = new PathBuilder();
        PathChain path = builder.addPath(
                new Path(
                        new BezierLine(
                                new Point(currentPose),
                                new Point(currentPose.getX(), currentPose.getY()+distance, Point.CARTESIAN)
                        )
                )).setConstantHeadingInterpolation(0).build();
        return path;
    }
}
