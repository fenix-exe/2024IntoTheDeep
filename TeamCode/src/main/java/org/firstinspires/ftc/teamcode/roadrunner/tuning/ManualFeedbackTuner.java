package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.*;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.Vector;

@TeleOp
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(-35, 42, -90));

            waitForStart();
           while (opModeIsActive()) {
               Actions.runBlocking(
                       drive.actionBuilder(new Pose2d(-35, 42, -90))
                               .strafeToLinearHeading(new Vector2d(-5.39, 31.96), Math.toRadians(-90.00))
                               .strafeToLinearHeading(new Vector2d(-34.84, 32.34), Math.toRadians(-90.00))
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

           }
        }else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}