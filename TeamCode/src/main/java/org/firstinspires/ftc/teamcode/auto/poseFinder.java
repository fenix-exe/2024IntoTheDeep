package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Config
@Autonomous
public class poseFinder extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;
    TelemetryPacket p;



    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(x, y, Math.toRadians(heading)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        p = new TelemetryPacket();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("pose x", drive.pose.position.toString());
            telemetry.addData("pose head", drive.pose.heading.toDouble());
            telemetry.update();


            Canvas c = p.fieldOverlay();


            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, drive.pose);
        }

    }
}
