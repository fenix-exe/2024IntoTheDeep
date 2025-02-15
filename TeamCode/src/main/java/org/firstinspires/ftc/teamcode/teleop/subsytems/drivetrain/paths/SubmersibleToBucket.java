package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AscentSideSubmersibleToBucketBlueAlliance extends BasePath{
    boolean lockPath = false;

    public AscentSideSubmersibleToBucketBlueAlliance(Pose currentPose) {
        super(currentPose);
    }

    @Override
    public PathChain getPathChain() {
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN), //starting point near submersible
                                new Point(currentPose.getX(), 125, Point.CARTESIAN),
                                new Point(36.290, 109.641, Point.CARTESIAN),
                                new Point(15, 125, Point.CARTESIAN) //end point
                        )
                )
                .setTangentHeadingInterpolation()//heading change
                .setReversed(true)
                .build();
        return paths;
    }

    @Override
    public void lockPath() {
        lockPath = true;
    }
}
