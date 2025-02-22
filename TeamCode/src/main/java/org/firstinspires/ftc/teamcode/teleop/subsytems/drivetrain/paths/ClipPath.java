package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class ClipPath extends BasePath{
    static ClipPath instance = null;
    public static int amountOfClips;

    private ClipPath() {
    }
    public static ClipPath getInstance(){
        if (instance==null){
            instance=new ClipPath();
        }
        return instance;
    }
    /*@Override
    public PathChain getPathChain() {
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN), //starting point near submersible
                                new Point(currentPose.getX(), amountOfClips * 2 + 70, Point.CARTESIAN),
                                new Point(32, amountOfClips * 2 + 70, Point.CARTESIAN) //end point
                        )
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(),0)//heading change
                .build();
        return paths;
    }*/

    @Override
    public PathChain getPathChain(Pose currentPose) {
        PathBuilder builder = new PathBuilder();
        ArrayList<Point> pointList = new ArrayList<Point>();
        Point startPoint = new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN);
        pointList.add(startPoint);
        //pointList.add(new Point(24,31,Point.CARTESIAN));
        for(int i = 0; i < controlPoints.size(); i++){
            pointList.add(convertToPoint(controlPoints.get(i), currentPose));
        }
        pointList.add(convertToPoint(endPointAsString, currentPose));

        PathBuilder pathsBuilt = builder.addPath(
                !controlPoints.isEmpty() ? new BezierCurve(pointList) :
                        new BezierLine(startPoint, convertToPoint(endPointAsString,currentPose))
        );

        if (interpolationType == null || interpolationType.equals("Constant")){
            pathsBuilt.setConstantHeadingInterpolation(Math.toRadians(interpolationParam1));
        } else if(interpolationType.equals("Linear")) {
            pathsBuilt.setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(interpolationParam1));
        } else {
            pathsBuilt.setTangentHeadingInterpolation();
        }
        pathsBuilt.addPath(new BezierLine(convertToPoint(endPointAsString,currentPose), new Point(36.5,72,Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(interpolationParam1))
                .setZeroPowerAccelerationMultiplier(2);
        return pathsBuilt.build();
    }
    public void increaseAmountOfClips(){
        amountOfClips += 1;
    }
}
