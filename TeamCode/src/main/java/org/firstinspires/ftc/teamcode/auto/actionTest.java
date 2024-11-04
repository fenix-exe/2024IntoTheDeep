package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;

@Autonomous
public class actionTest extends LinearOpMode {
    CRServo intake;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0,0,180);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        activeIntake activeIntake = new activeIntake(intake);
        intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        Actions.runBlocking(
                activeIntake.aIForward()

        );

    }
}
