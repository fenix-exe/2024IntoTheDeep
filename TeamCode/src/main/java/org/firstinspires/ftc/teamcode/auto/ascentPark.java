package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.extractAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

@Autonomous(name = "AUTO - Ascent Park")
public class ascentPark extends LinearOpMode {
    String filename = "/sdcard/Download/autoPositions/ascentPark.csv";
    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            vector = extractAuto.SetUpListOfThings(telemetry, filename );
        } catch (FileNotFoundException e) {
            telemetry.addData("No File Detected. File name is:", filename);
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

        boolean XareSame = false;
        boolean YareSame = false;
        boolean AngleareSame = false;
        for (int i = 1; i < vector.size(); i++) {
            XareSame = ((extractAuto.getXFromList(vector.get(i-1)) == extractAuto.getXFromList(vector.get(i))));
            YareSame = ((extractAuto.getYFromList(vector.get(i-1)) == extractAuto.getYFromList(vector.get(i))));
            AngleareSame = ((extractAuto.getAngleFromList(vector.get(i-1)) == extractAuto.getAngleFromList(vector.get(i))));
            if (!XareSame && YareSame && AngleareSame) {
                traj1 = traj1.lineToX(extractAuto.getXFromList(vector.get(i)));
            }
            if (XareSame && !YareSame && AngleareSame) {
                traj1 = traj1.lineToY(extractAuto.getYFromList(vector.get(i)));
            }
            if (XareSame && YareSame && !AngleareSame) {
                traj1 = traj1.turnTo(extractAuto.getAngleFromList(vector.get(i)));
            }
            if (!XareSame && !YareSame && AngleareSame) {
                traj1 = traj1.splineToConstantHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))), extractAuto.getAngleFromList(vector.get(i)));
            } else {
                traj1 = traj1.splineTo(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))), extractAuto.getAngleFromList(vector.get(i)));
            }
            telemetry.addData("Vector " + (i) + " X", extractAuto.getXFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Y", extractAuto.getYFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Heading", extractAuto.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Elbow Phi", extractAuto.getElbowPhiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Linear Slide", extractAuto.getLinearSlideFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wrist Psi", extractAuto.getWristPsiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wrist Rho", extractAuto.getWristRhoFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Intake", extractAuto.getIntakeFromList(vector.get(i)));
        }

        Action action1 = traj1.build();
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(action1);

    }
}
