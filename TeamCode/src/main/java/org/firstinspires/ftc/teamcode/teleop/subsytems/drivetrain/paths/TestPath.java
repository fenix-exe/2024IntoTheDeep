package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class TestPath extends BasePath{
    static TestPath instance = null;
    private TestPath() {
        super();
    }
    public static TestPath getInstance(){
        if (instance==null){
            instance=new TestPath();
        }
        return instance;
    }
    @Override
    public PathChain getPathChain(Pose currentPose) {
        PathBuilder builder = new PathBuilder();
        ArrayList<Point> pointList = new ArrayList<Point>();
        pointList.add(new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN));
        for(int i = 0; i < controlPoints.size(); i++){
            pointList.add(convertToPoint(controlPoints.get(i), currentPose));
        }
        pointList.add(convertToPoint(endPointAsString, currentPose));
        /*PathChain paths = builder.addPath(
                        !controlPoints.isEmpty() ?
                        new BezierCurve(pointList) :
                        new BezierLine(new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN), convertToPoint(endPointAsString,currentPose))
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(interpolationParam1))
                .build();*/

        PathBuilder pathsBuilt = builder.addPath(
                !controlPoints.isEmpty() ? new BezierCurve(pointList) :
                        new BezierLine(new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN), convertToPoint(endPointAsString,currentPose))
        );

        if (interpolationType == null || interpolationType.equals("Constant")){
            pathsBuilt.setConstantHeadingInterpolation(Math.toRadians(interpolationParam1));
        } else if(interpolationType.equals("Linear")) {
            pathsBuilt.setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(interpolationParam1));
        } else {
            pathsBuilt.setTangentHeadingInterpolation();
        }

        return pathsBuilt.build();
    }
}
