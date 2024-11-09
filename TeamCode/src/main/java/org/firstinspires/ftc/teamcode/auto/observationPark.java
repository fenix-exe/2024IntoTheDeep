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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.util.extractAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


@Autonomous(name = "AUTO - Observation Park")
public class observationPark extends LinearOpMode {

    //initialize auto extractor
    String filename = "/sdcard/Download/autoPositions/observationPark.csv";
    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();

    CRServo intake;
    Servo left;
    Servo right;

    @Override
    public void runOpMode() throws InterruptedException {
        //add telemetry to FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //try to read and extract data from file
        try {
            vector = extractAuto.SetUpListOfThings(telemetry, filename );
        } catch (FileNotFoundException e) {
            telemetry.addData("No File Detected. File name is:", filename);
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        //set up rr
        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = hardwareMap.get(CRServo.class, "intake");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        activeIntake activeIntake = new activeIntake(intake);
        differential diffy = new differential(left, right);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

        //build trajectory based on file data
        for (int i = 1; i < vector.size(); i++) {
            traj1 = traj1.splineToConstantHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))), extractAuto.getAngleFromList(vector.get(i)));
            activeIntake.aIControl(extractAuto.getXFromList(vector.get(i)));
            diffy.setDiffy(extractAuto.getWristPsiFromList(vector.get(i)), extractAuto.getWristRhoFromList(vector.get(i)));
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
