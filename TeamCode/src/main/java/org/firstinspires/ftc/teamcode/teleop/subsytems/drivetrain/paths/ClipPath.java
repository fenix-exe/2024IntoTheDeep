package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class ClipPath extends BasePath{
    boolean lockPath = false;
    public static int amountOfClips;

    public ClipPath(Pose currentPose) {
        super(currentPose, new Pose(26,amountOfClips * 2 + 70,0));
    }

    @Override
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
    }

    @Override
    public void lockPath() {
        lockPath = true;
    }
}
