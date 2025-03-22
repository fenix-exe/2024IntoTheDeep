package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive;

import java.io.File;
import java.util.Arrays;

@Config
@TeleOp(name = "POSE FINDER")
public class poseFinder extends LinearOpMode {



    public static double x = 39.7;
    public static double y = 65;
    public static double heading = -180;
    TelemetryPacket p;



    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(x, y, Math.toRadians(heading)));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();

        waitForStart();


        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("pose x", drive.pose.position.x);
            telemetry.addData("pose y", drive.pose.position.y);
            telemetry.addData("pose head", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, drive.pose);
            if (gamepad1.a) {
                Arrays.stream(new File("/sdcard/Download/autoLogger").listFiles()).forEach(File::delete);
            }
        }

    }
}
