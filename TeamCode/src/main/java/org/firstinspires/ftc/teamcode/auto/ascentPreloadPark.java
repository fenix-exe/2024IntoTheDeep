package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.fullClaw;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;
import org.firstinspires.ftc.teamcode.util.autoTeleTransfer;
import org.firstinspires.ftc.teamcode.util.extractAuto;
import org.firstinspires.ftc.teamcode.util.writeAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


@Autonomous(name = "AUTO - Ascent Park", preselectTeleOp = "TeleOPV4")
public class ascentPreloadPark extends LinearOpMode {

    //initialize auto extractor
    String filename = "/sdcard/Download/autoPositions/ascentPreloadPark.csv";
    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();

    ServoImplEx pitch;
    ServoImplEx roll;
    ServoImplEx claw;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DcMotorEx slide;
    DcMotorEx elbow;
    writeAuto writer;
    RevTouchSensor limitSwitch;
    fullClaw fullClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        //add telemetry to FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        writer = new writeAuto("/sdcard/Download/save.csv");

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


        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        fullClaw = new fullClaw(pitch, roll, claw);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        elbow = hardwareMap.get(DcMotorEx.class, "pivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");


        //Homing the pivot
        /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);*/

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideCode = new slideCodeFunctions(slide);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(elbow, pivotPIDF, 2178);



        while(!gamepad1.a) {

        }
        while (!limitSwitch.isPressed() && !isStopRequested()){
            elbow.setPower(-0.2);
        }
        while (limitSwitch.isPressed() && !isStopRequested()){
            elbow.setPower(0.2);
        }
        elbow.setPower(0);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!gamepad1.b) {

        }
        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

        boolean XareSame = false;
        boolean YareSame = false;
        boolean AngleareSame = false;
        //build trajectory based on file data
        for (int i = 1; i < vector.size(); i++) {
            XareSame = ((extractAuto.getXFromList(vector.get(i-1)) == extractAuto.getXFromList(vector.get(i))));
            YareSame = ((extractAuto.getYFromList(vector.get(i-1)) == extractAuto.getYFromList(vector.get(i))));
            AngleareSame = ((extractAuto.getAngleFromList(vector.get(i-1)) == extractAuto.getAngleFromList(vector.get(i))));
            if (XareSame && YareSame && AngleareSame) {
                traj1 = traj1.stopAndAdd(pivotCode.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i))))
                        .stopAndAdd(slideCode.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .stopAndAdd(fullClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));

                //Active Intake servo not working
            } else {
                traj1 = traj1.afterDisp(0,pivotCode.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i))))
                        .afterDisp(0,slideCode.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i)),extractAuto.getAngleFromList(vector.get(i))), Math.PI/2)
                        .stopAndAdd(fullClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));                //Active Intake servo not working
            }
            telemetry.addData("Vector " + (i) + " X", extractAuto.getXFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Y", extractAuto.getYFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Heading", extractAuto.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Elbow Phi", extractAuto.getElbowPhiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Linear Slide", extractAuto.getLinearSlideFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Pitch", extractAuto.getPitchFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Roll", extractAuto.getRollFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Claw", extractAuto.getClawFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wait", extractAuto.getWaitFromList(vector.get(i)));
            telemetry.update();

        }

        Action action1 = traj1.build();


        pivotCode.goTo(870);
        fullClaw.setPitch(0.7);
        fullClaw.setRoll(0);
        fullClaw.setClaw(1);
        if (870-30 < elbow.getCurrentPosition() && elbow.getCurrentPosition() < 870+30) {
            elbow.setPower(0);

        } else {
            pivotCode.goTo(870);
        }


        waitForStart();

        if (isStopRequested()) {
            return;
        }


        Actions.runBlocking(action1);




        autoTeleTransfer.setElbowTicks(elbow.getCurrentPosition());
        autoTeleTransfer.setSlideTicks(slide.getCurrentPosition());
        autoTeleTransfer.setxPos(extractAuto.getXFromList(vector.get(vector.size()-1)));
        autoTeleTransfer.setyPos(extractAuto.getYFromList(vector.get(vector.size()-1)));
        autoTeleTransfer.setHeadingAngle(extractAuto.getAngleFromList(vector.get(vector.size()-1)));


        return;
    }
}
