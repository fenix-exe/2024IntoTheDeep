package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class SubmersibleToBucketBlueAlliance extends BasePath{


    public SubmersibleToBucketBlueAlliance(Pose currentPose) {
        super(currentPose);
    }

    @Override
    public PathChain getPathChain() {
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN), //starting point near submersible
                                new Point(currentPose.getX(), 134.000, Point.CARTESIAN), //control point
                                new Point(9.500, 134.000, Point.CARTESIAN) //end point
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-32)) //heading change
                .build();
        return paths;
    }
}
