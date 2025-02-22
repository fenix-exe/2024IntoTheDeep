package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class ClipToHumanPlayer extends BasePath{

    static ClipToHumanPlayer instance = null;
    private ClipToHumanPlayer() {
        super();
    }
    public static ClipToHumanPlayer getInstance(){
        if (instance==null){
            instance=new ClipToHumanPlayer();
        }
        return instance;
    }
    @Override
    public PathChain getPathChain(Pose currentPose) {
        PathBuilder builder = new PathBuilder();
        ArrayList<Point> pointList = new ArrayList<Point>();
        Point startPoint = new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN);
        pointList.add(startPoint);
        for(int i = 0; i < controlPoints.size(); i++){
            pointList.add(convertToPoint(controlPoints.get(i), currentPose));
        }
        pointList.add(convertToPoint(endPointAsString, currentPose));
        PathBuilder paths = builder.addPath(
                        !controlPoints.isEmpty() ? new BezierCurve(pointList) :
                                new BezierLine(startPoint, convertToPoint(endPointAsString,currentPose))
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(interpolationParam1));
                //.setZeroPowerAccelerationMultiplier(0.3)
        paths.addPath(new BezierLine(convertToPoint(endPointAsString, currentPose), new Point(21.5, 28.5, Point.CARTESIAN))).setConstantHeadingInterpolation(Math.PI);
        return paths.build();
    }
}
