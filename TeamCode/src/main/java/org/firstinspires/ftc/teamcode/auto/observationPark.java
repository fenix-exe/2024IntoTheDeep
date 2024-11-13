package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;
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
    ServoImplEx left;
    ServoImplEx right;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DcMotorEx slide;
    DcMotorEx pivot;

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
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        activeIntake activeIntake = new activeIntake(intake);
        differential diffy = new differential(left, right);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        //Homing the pivot
        /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);*/

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideCode = new slideCodeFunctions(slide);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, 2178);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

        //build trajectory based on file data
        for (int i = 1; i < vector.size(); i++) {
            traj1 = traj1.splineToConstantHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))), extractAuto.getAngleFromList(vector.get(i)))
                    .stopAndAdd(slideCode.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                    .stopAndAdd(diffy.setDiffy(extractAuto.getWristPsiFromList(vector.get(i)),extractAuto.getWristRhoFromList(vector.get(i))));
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
