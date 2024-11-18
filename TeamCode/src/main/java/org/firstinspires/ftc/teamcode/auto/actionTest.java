package org.firstinspires.ftc.teamcode.auto;

import android.widget.Switch;

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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;
import org.firstinspires.ftc.teamcode.util.extractAuto;
import org.firstinspires.ftc.teamcode.util.writeAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

@Autonomous
public class actionTest extends LinearOpMode {
    CRServo intake;
    ServoImplEx left;
    ServoImplEx right;
    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DcMotorEx slide;
    DcMotorEx pivot;
    writeAuto writer;
    //TouchSensor limitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        extractAuto extractAuto = new extractAuto();
        ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();

        String filename;
        String save = "/sdcard/Download/save.csv";

        intake = hardwareMap.get(CRServo.class, "intake");
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
        activeIntake activeIntake = new activeIntake(intake);

        differential diffy = new differential(left, right);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        writer = new writeAuto(save);

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

        int currentNum = 1;

        int selectedNum = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        while (selectedNum !=2 || selectedNum !=3 || selectedNum !=4 || selectedNum !=5 && !isStopRequested()){
            previousGamepad1.copy(gamepad1);
            telemetry.addLine("1 - Home Pivot");
            telemetry.addLine("2 - AUTO - Observation Park");
            telemetry.addLine("3 - AUTO - Observation Preload Park");
            telemetry.addLine("4 - AUTO - Ascent Preload Park");
            telemetry.addLine("5 - AUTO - Ascent Cycle Park");
            telemetry.addData("Selected Number", currentNum);
            telemetry.update();
            if (gamepad1.a && !previousGamepad1.a){
                if (currentNum == 5) {
                    currentNum = 1;
                } else {
                    currentNum++;
                }
            }
            if (gamepad1.start && !previousGamepad1.start){
                selectedNum = currentNum;
                telemetry.addLine("intialized");
                telemetry.update();
                break;
            }
            if (selectedNum == 1){
                /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
                }*/
                pivot.setPower(0);
                pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                selectedNum = 0;
            } else {
                pivot.setPower(0);
            }
            currentGamepad1.copy(gamepad1);
        }
        switch (selectedNum){
            case 2:
                filename = "/sdcard/Download/autoPositions/observationPark.csv";
                break;
            case 3:
                filename = "/sdcard/Download/autoPostions/observationPreloadPark.csv";
                break;
            case 4:
                filename = "/sdcard/Download/autoPositions/ascentPreloadPark.csv";
                break;
            case 5:
                filename = "/sdcard/Download/autoPositions/ascentPreloadCyclePark.csv";
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + selectedNum);
        }

        telemetry.addLine("read file");
        telemetry.update();

        try {
            vector = extractAuto.SetUpListOfThings(telemetry, filename);
        } catch (FileNotFoundException e) {
            telemetry.addData("No File Detected. File name is:", filename);
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        telemetry.addLine("setup");
        telemetry.update();

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
                        .stopAndAdd(diffy.setDiffy(extractAuto.getWristPsiFromList(vector.get(i)),extractAuto.getWristRhoFromList(vector.get(i))))
                        .stopAndAdd(activeIntake.aIControl(extractAuto.getIntakeFromList(vector.get(i))))
                        .stopAndAdd(writer.savePosition(pivotCode.getElbowTicks(),slideCode.getSlidePosition(),extractAuto.getWristPsiFromList(vector.get(i)),extractAuto.getWristRhoFromList(vector.get(i))))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            } else {
                traj1 = traj1.splineToConstantHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)), extractAuto.getYFromList(vector.get(i))), extractAuto.getAngleFromList(vector.get(i)))
                        .stopAndAdd(pivotCode.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i))))
                        .stopAndAdd(slideCode.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .stopAndAdd(diffy.setDiffy(extractAuto.getWristPsiFromList(vector.get(i)), extractAuto.getWristRhoFromList(vector.get(i))))
                        .stopAndAdd(activeIntake.aIControl(extractAuto.getIntakeFromList(vector.get(i))))
                        .stopAndAdd(writer.savePosition(pivotCode.getElbowTicks(),slideCode.getSlidePosition(),extractAuto.getWristPsiFromList(vector.get(i)),extractAuto.getWristRhoFromList(vector.get(i))))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
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

        telemetry.update();


        Action action1 = traj1.build();

        pivotCode.goTo(extractAuto.getElbowPhiFromList(vector.get(0)));
        slideCode.goTo(extractAuto.getLinearSlideFromList(vector.get(0)));
        diffy.setDifferentialPosition(extractAuto.getWristPsiFromList(vector.get(0)), extractAuto.getWristRhoFromList(vector.get(0)));



        waitForStart();


        if (isStopRequested()) return;


        telemetry.addData("Status", "Running action1");
        telemetry.update();

        Actions.runBlocking(action1
                );

        telemetry.addData("Status", "Completed action1");
        telemetry.addData("loxa", left.getPosition());
        telemetry.update();

    }
}
