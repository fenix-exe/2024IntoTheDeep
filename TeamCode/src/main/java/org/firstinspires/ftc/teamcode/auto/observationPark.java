package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ListOfThings;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

@Autonomous(name = "AUTO - Observation Park")
public class observationPark extends LinearOpMode {
    String filename = "/sdcard/Download/autoPositions/observationPark.csv";
    ListOfThings listOfThings = new ListOfThings();
    ArrayList<ListOfThings.PositionInSpace> vector = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            vector = listOfThings.SetUpListOfThings(telemetry, filename );
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        Pose2d beginPose = new Pose2d(listOfThings.getXFromList(vector.get(0)), listOfThings.getYFromList(vector.get(0)), listOfThings.getAngleFromList(vector.get(0)));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

        for (int i = 1; i < vector.size(); i++) {
            traj1 = traj1.splineToConstantHeading(new Vector2d(listOfThings.getXFromList(vector.get(i)), listOfThings.getYFromList(vector.get(i))), listOfThings.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " X", listOfThings.getXFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Y", listOfThings.getYFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Heading", listOfThings.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Elbow Phi", listOfThings.getElbowPhiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Linear Slide", listOfThings.getLinearSlideFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wrist Psi", listOfThings.getWristPsiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wrist Rho", listOfThings.getWristRhoFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Intake", listOfThings.getIntakeFromList(vector.get(i)));
        }
        Action action1 = traj1.build();
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(action1);

    }
}
