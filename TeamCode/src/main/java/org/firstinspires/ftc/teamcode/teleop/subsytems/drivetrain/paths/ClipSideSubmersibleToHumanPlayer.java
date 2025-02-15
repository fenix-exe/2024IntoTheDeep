package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class ClipSideSubmersibleToHumanPlayer extends BasePath{
    boolean lockPath = false;

    public ClipSideSubmersibleToHumanPlayer(Pose currentPose) {
        super(currentPose,null);
    }

    @Override
    public PathChain getPathChain() {
        PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(41.115, 70.842, Point.CARTESIAN),
                                new Point(11.196, 55.786, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(),-90)//heading change
                .setReversed(true)
                .build();
        return paths;
    }

    @Override
    public void lockPath() {
        lockPath=true;
    }
}
