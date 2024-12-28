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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.claw.autoClaw;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.util.extractAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


@Autonomous(name = "AUTO - Ascent Clip Park", preselectTeleOp = "TeleOPV4")
public class ascentClipCyclePark extends LinearOpMode {

    //initialize auto extractor
    String FILE_NAME = "/sdcard/Download/autoPositions/ascentClipCyclePark.csv";
    int ELBOW_START = 870;
    int SLIDE_START = 0;
    double PITCH_START = 0.5;
    double ROLL_START = 0;
    double CLAW_START = 1;


    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();

    ServoImplEx pitch;
    ServoImplEx roll;
    ServoImplEx claw;

    Elbow elbow;
    DcMotorEx elbowMotor;

    PIDController controllerPivotPIDF;

    Slide slide;
    DcMotorEx slideMotor;

    RevTouchSensor limitSwitch;
    autoClaw autoClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        //add telemetry to FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //try to read and extract data from file
        try {
            vector = extractAuto.SetUpListOfThings(telemetry, FILE_NAME);
        } catch (FileNotFoundException e) {
            telemetry.addData("No File Detected. File name is:", FILE_NAME);
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        //set up rr


        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        autoClaw = new autoClaw(pitch, roll, claw);

        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Homing the pivot
        /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);*/



        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide = new Slide(slideMotor, 3000);


        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        Elbow elbow = new Elbow(elbowMotor, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2500);



        while(!gamepad1.a && !isStopRequested()) {

        }

        while (!limitSwitch.isPressed() && !isStopRequested()){
            elbowMotor.setPower(-0.2);
        }
        while (limitSwitch.isPressed() && !isStopRequested()){
            elbowMotor.setPower(0.2);
        }
        elbowMotor.setPower(0);

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!gamepad1.b && !isStopRequested()) {

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
                traj1 = traj1.stopAndAdd(elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .stopAndAdd(slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));

                //Active Intake servo not working
            }
            if (!XareSame && YareSame && AngleareSame) {
                traj1.afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .strafeTo(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }
            if (XareSame && !YareSame && AngleareSame) {
                traj1.afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .strafeTo(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }
            if (XareSame && YareSame && !AngleareSame) {
                traj1.afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .turnTo(extractAuto.getAngleFromList(vector.get(i)))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i)) ))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }
            else {
                traj1 = traj1.afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i)),extractAuto.getAngleFromList(vector.get(i))), Math.PI/2)
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));                //Active Intake servo not working
            }
            telemetry.addData("Vector " + (i) + " X", extractAuto.getXFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Y", extractAuto.getYFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Heading", extractAuto.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Elbow Phi", extractAuto.getElbowPhiFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Elbow Speed", extractAuto.getElbowSpeedFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Linear Slide", extractAuto.getLinearSlideFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Pitch", extractAuto.getPitchFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Roll", extractAuto.getRollFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Claw", extractAuto.getClawFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Wait", extractAuto.getWaitFromList(vector.get(i)));
            telemetry.update();

        }

        Action action1 = traj1.build();


        elbow.goTo(ELBOW_START, 1);
        autoClaw.setPitch(PITCH_START);
        autoClaw.setRoll(ROLL_START);


        if (ELBOW_START-30 < elbowMotor.getCurrentPosition() && elbowMotor.getCurrentPosition() < ELBOW_START+30) {
            elbowMotor.setPower(0);

        } else {
            elbow.goTo(ELBOW_START, 1);
        }

        while(!gamepad1.y && !isStopRequested()) {

        }

        autoClaw.setClaw(CLAW_START);


        waitForStart();

        if (isStopRequested()) {
            return;
        }


        Actions.runBlocking(action1);






        return;
    }
}
