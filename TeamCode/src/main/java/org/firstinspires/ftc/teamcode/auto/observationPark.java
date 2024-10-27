package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class observationPark extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12,60, Math.toRadians(180)));
        Action traj1 = drive.actionBuilder(new Pose2d(-12,60, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-66, 55), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking((traj1));

    }
}
