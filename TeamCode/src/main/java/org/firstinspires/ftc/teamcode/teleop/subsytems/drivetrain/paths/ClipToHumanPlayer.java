package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class ClipToHumanPlayer extends BasePath{
    boolean lockPath = false;
    public ClipToHumanPlayer(Pose currentPose) {
        super(currentPose, new Pose(14,24,Math.PI));
    }

    @Override
    public PathChain getPathChain() {
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN),
                                new Point(currentPose.getX(), 24, Point.CARTESIAN),
                                new Point(14, currentPose.getY(), Point.CARTESIAN),
                                new Point(44,24,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.PI)//heading change
                .build();
        return paths;
    }

    @Override
    public void lockPath() {
        lockPath=true;
    }
}
