package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class SubmersibleToBucket extends BasePath{
    static SubmersibleToBucket instance = null;

    private SubmersibleToBucket() {
        super();
    }
    public static SubmersibleToBucket getInstance(){
        if (instance==null){
            instance=new SubmersibleToBucket();
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
        PathChain paths = builder.addPath(
                new BezierCurve(pointList)
        )
        .setTangentHeadingInterpolation()
        .setReversed(true)
        .build();
        return paths;
    }
}
