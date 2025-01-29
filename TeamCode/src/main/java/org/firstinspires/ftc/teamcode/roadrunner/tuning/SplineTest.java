package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@TeleOp
public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-4, 69, Math.toRadians(-90));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-5.39, 42), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-34.84, 42), Math.toRadians(-90.00))
                        .splineToLinearHeading(new Pose2d(-42.55, 11.94, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-53.71, 63.72), Math.toRadians(-90.00))
                        .splineToLinearHeading(new Pose2d(-54.10, 15.79, Math.toRadians(-90.00)), Math.toRadians(-86.98))
                        .strafeToLinearHeading(new Vector2d(-61.03, 64.88), Math.toRadians(-90.00))
                        .splineToLinearHeading(new Pose2d(-63.34, 12.13, Math.toRadians(-90.00)), Math.toRadians(267.79))
                        .strafeToLinearHeading(new Vector2d(-68.92, 58.14), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-41.01, 64.88), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-3.66, 26.95), Math.toRadians(-46.02))
                        .strafeToLinearHeading(new Vector2d(-41.01, 64.88), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-3.66, 26.95), Math.toRadians(-46.02))
                        .strafeToLinearHeading(new Vector2d(-41.01, 64.88), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-3.66, 26.95), Math.toRadians(-46.02))
                        .strafeToLinearHeading(new Vector2d(-41.01, 64.88), Math.toRadians(-90.00))
                        .strafeToLinearHeading(new Vector2d(-3.66, 26.95), Math.toRadians(-46.02))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
