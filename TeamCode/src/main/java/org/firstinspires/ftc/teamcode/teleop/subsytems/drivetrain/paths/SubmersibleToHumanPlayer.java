package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class SubmersibleToHumanPlayer extends BasePath{
    static SubmersibleToHumanPlayer instance = null;
    private SubmersibleToHumanPlayer() {
        super();
    }
    public static SubmersibleToHumanPlayer getInstance(){
        if (instance==null){
            instance=new SubmersibleToHumanPlayer();
        }
        return instance;
    }
    @Override
    public PathChain getPathChain(Pose currentPose) {
        ArrayList<Point> pointList = new ArrayList<Point>();
        pointList.add(new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN));
        for(int i = 0; i < controlPoints.size(); i++){
            pointList.add(convertToPoint(controlPoints.get(i), currentPose));
        }
        pointList.add(convertToPoint(endPointAsString, currentPose));
        PathChain paths = builder.addPath(
                        new BezierCurve(pointList)
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(interpolationParam1))
                .build();
        return paths;
    }
}
