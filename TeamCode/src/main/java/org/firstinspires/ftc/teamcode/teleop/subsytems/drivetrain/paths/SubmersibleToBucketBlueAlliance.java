package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public class SubmersibleToBucketBlueAlliance extends BasePath{

    public SubmersibleToBucketBlueAlliance(Pose currentPose) {
        super(currentPose);
    }

    @Override
    public PathChain getPathChain() {
        ArrayList<Point> pointList = (ArrayList<Point>) controlPoints.clone();
        pointList.add(0, new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN));
        pointList.add(endPoint);
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(pointList)
                )
                .setLinearHeadingInterpolation(Math.toRadians(interpolationParam1), Math.toRadians(interpolationParam2)) //heading change
                .build();
        return paths;
    }
}
