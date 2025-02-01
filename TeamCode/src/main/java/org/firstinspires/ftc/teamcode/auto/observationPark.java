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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsytems.claw.autoClaw;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.util.RobotWideFunctions;
import org.firstinspires.ftc.teamcode.util.extractAuto;
import org.firstinspires.ftc.teamcode.util.writeAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


@Autonomous(name = "AUTO - OBSERVATION PARK!!!", preselectTeleOp = "TeleOpV5Sample")
public class observationPark extends LinearOpMode {

    //initialize auto extractor
    String FILE_NAME = "/sdcard/Download/autoPositions/observationPark.csv";
    int ELBOW_START = 45;
    int SLIDE_START = 0;
    double PITCH_START = 0.8;
    double ROLL_START = 0.2;
    double CLAW_START = 1;


    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();
    RobotWideFunctions robot = new RobotWideFunctions();

    ServoImplEx pitch;
    ServoImplEx roll;
    ServoImplEx claw;
    autoClaw autoClaw;

    Elbow elbow;
    DcMotorEx elbowMotor;
    RevTouchSensor limitSwitch;

    ElapsedTime timer;

    PIDController controllerPivotPIDF;

    DcMotorEx slideMotor;
    Slide slide;
    RevTouchSensor homingSwitch;


    DcMotorEx linearActuatorMotor;
    RevTouchSensor actuatorSwitch;
    LinearActuator linearActuator;


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

        writeAuto writer = new writeAuto("ascentPreloadTIme");

        //set up rr


        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
        pitch.setDirection(Servo.Direction.REVERSE);
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        autoClaw = new autoClaw(pitch, roll, claw);


        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");
        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);




        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");


        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        elbow = new Elbow(elbowMotor, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2500);


        while (!gamepad1.a) {

        }

        //HOMING
        pitch.setPosition(0.5);

        while (!slide.isHomingSwitchPressed() && !isStopRequested()){
            slide.setSlidePower(-0.2);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Homing the elbow
        while (!limitSwitch.isPressed() && !isStopRequested()){
            elbow.setElbowPower(-0.2);
        }
        while (limitSwitch.isPressed() && !isStopRequested()){
            elbow.setElbowPower(-0.4);
        }
        while (!limitSwitch.isPressed() && !isStopRequested()){
            elbow.setElbowPower(0.4);
        }

        elbow.setElbowPower(0);

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //homing the linear actuator
        while (!linearActuator.getLimitSwitchState() && !isStopRequested()){
            linearActuator.setLinearActuatorPower(-0.5);
        }
        linearActuator.setLinearActuatorPower(0);

        linearActuator.resetEncoders();

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!gamepad1.b && !isStopRequested()) {

        }


        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
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
                //This is when the robot does not move.
                traj1 = traj1
                        .stopAndAdd(elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .stopAndAdd(slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))))
                        .stopAndAdd(robot.vectorLog(i,telemetry))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }

            else if (XareSame && YareSame && !AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .turnTo(extractAuto.getAngleFromList(vector.get(i)))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))))
                        .stopAndAdd(robot.vectorLog(i,telemetry))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }
            else if ((!XareSame || !YareSame) && AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))))
                        .stopAndAdd(robot.vectorLog(i,telemetry))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }
            else {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)))
                        .stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))))
                        .stopAndAdd(robot.vectorLog(i,telemetry))
                        .waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
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


        elbow.goTo(elbow.degreesToTicks(ELBOW_START), 1);
        autoClaw.setPitch(PITCH_START);
        autoClaw.setRoll(ROLL_START);


        if (ELBOW_START-30 < elbowMotor.getCurrentPosition() && elbowMotor.getCurrentPosition() < ELBOW_START+30) {
            elbowMotor.setPower(0);

        } else {
            elbow.goTo(ELBOW_START, 1);
        }

        while(!gamepad1.y && !isStopRequested()) {

        }

        elbowMotor.setPower(0);

        autoClaw.setClaw(CLAW_START);

        ElapsedTime timer = new ElapsedTime();


        waitForStart();
        timer.reset();

        if (isStopRequested()) {
            return;
        }


        Actions.runBlocking(action1);

        writer.timer(timer.time());






        return;
    }
}
