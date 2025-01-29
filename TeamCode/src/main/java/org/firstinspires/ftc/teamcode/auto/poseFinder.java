package org.firstinspires.ftc.teamcode.auto;

import android.os.FileUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.io.File;
import java.util.Arrays;

@Config
@TeleOp(name = "POSE FINDER")
public class poseFinder extends LinearOpMode {



    public static double x = -4;
    public static double y = 69;
    public static double heading = -90;
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
