package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import page.j5155.expressway.ftc.actions.ActionLinearOpMode;
import page.j5155.expressway.ftc.actions.ActionOpMode;
import page.j5155.expressway.ftc.actions.ActionRunner;
import page.j5155.expressway.ftc.motion.PIDToPoint;

@TeleOp
public class AutoTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        Pose2d target = new Pose2d(0, 6, 0);
        Pose2d home = new Pose2d(24,0,0);
        MecanumDrive.PIDDrive targetMove = drive.pidToPointAction(target);
        MecanumDrive.PIDDrive homeMove = drive.pidToPointAction(home);
        TelemetryPacket p = new TelemetryPacket();
        ActionRunner runner = new ActionRunner();

        waitForStart();
        while (opModeIsActive()) {
            runner.runAsync(targetMove);
            runner.updateAsync();
            telemetry.addData("pin",drive.pose.position.y);
            telemetry.update();
        }
    }
}
