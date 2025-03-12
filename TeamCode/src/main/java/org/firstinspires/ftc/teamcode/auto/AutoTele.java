package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import page.j5155.expressway.ftc.actions.ActionLinearOpMode;
import page.j5155.expressway.ftc.actions.ActionOpMode;
import page.j5155.expressway.ftc.actions.ActionRunner;
import page.j5155.expressway.ftc.motion.PIDToPoint;

import static page.j5155.expressway.core.geometry.GeometryHelpers.distanceTo;

@TeleOp
public class AutoTele extends LinearOpMode {
    PinpointDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        TelemetryPacket p = new TelemetryPacket();
        ActionRunner runner = new ActionRunner();


        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");

        MecanumDrive.PIDDrive targetmove = drive.pidToPointAction(new Pose2d(0, 40, Math.toRadians(90)), telemetry);
        MecanumDrive.PIDDrive homemove = drive.pidToPointAction(new Pose2d(0, 0, Math.toRadians(0)), telemetry);


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*TrajectoryActionBuilder traj1 = drive.actionBuilder(new Pose2d(48, -48, Math.toRadians(180)))
                .stopAndAdd(drive.pidToPointAction(new Pose2d(-48, -48, Math.toRadians(180)), telemetry))
                .stopAndAdd(drive.pidToPointAction(new Pose2d(-48, 48, Math.toRadians(180)), telemetry))
                .stopAndAdd(drive.pidToPointAction(new Pose2d(48, 48, Math.toRadians(180)), telemetry))
                .stopAndAdd(drive.pidToPointAction(new Pose2d(48, -48, Math.toRadians(180)), telemetry));*/




        //Action action1 = traj1.build();

        waitForStart();
        while (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(targetmove));
            sleep(1000);
            Actions.runBlocking(new SequentialAction(homemove));
            sleep(1000);
        }
    }
}
















































































































