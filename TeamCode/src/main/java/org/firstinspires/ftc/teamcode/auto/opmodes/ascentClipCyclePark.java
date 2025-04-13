package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.subsystems.activeIntake.autoBigWheelIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.auto.subsystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.common.util.Homing;
import org.firstinspires.ftc.teamcode.auto.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.auto.subsystems.claw.autoClaw;
import org.firstinspires.ftc.teamcode.auto.subsystems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.auto.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.auto.util.RobotWideFunctions;
import org.firstinspires.ftc.teamcode.auto.util.extractAuto;
import org.firstinspires.ftc.teamcode.auto.util.writeAuto;
import org.firstinspires.ftc.teamcode.teleop.util.testCode.homing.HomeTeleOpDown;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;


@Autonomous(name = "AUTO - CLIP 5", preselectTeleOp = "TeleOpV5SampleZapdos")
public class ascentClipCyclePark extends LinearOpMode {

    //declare vars
    String FILE_NAME = "/sdcard/Download/autoPositions/ascentClipCyclePark.csv";
    String LOG_NAME = "ascentClipCyclePark";
    //double ROLL_START = 0.21;
    //double CLAW_START = 0.86;

    //initialize interpreter
    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();
    RobotWideFunctions robot = new RobotWideFunctions();



    //declare end effector
    ServoImplEx pitchLeft;
    //ServoImplEx pitchRight;
    //ServoImplEx roll;
    CRServoImplEx leftRoller;
    CRServoImplEx rightRoller;
    autoClaw autoClaw;
    Wrist wrist;
    Claw clawCode;
    RevColorSensorV3 color;

    // declare elbow
    Elbow elbow;
    DcMotorEx elbowMotor;
    RevTouchSensor elbowSwitch;

    //set up slides
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    Slide slide;
    RevTouchSensor slideSwitch;



    //set up linear actuator
    DcMotorEx linearActuatorMotor;
    RevTouchSensor actuatorSwitch;
    LinearActuator linearActuator;

    //set up homing agent
    Homing homingAgent;
    public GoBildaPinpointDriverRR pinpoint;


    @Override
    public void runOpMode() throws InterruptedException {
        //add telemetry to FTC dashboard
        //MultipleTelemetry p = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //try to read and extract data from file
        try {
            vector = extractAuto.SetUpListOfThings(telemetry, FILE_NAME);
        } catch (FileNotFoundException e) {
            telemetry.addData("No File Detected. File name is:", FILE_NAME);
            telemetry.update();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        double ELBOW_START =extractAuto.getElbowPhiFromList(vector.get(0));
        double PITCH_START = extractAuto.getPitchFromList(vector.get(0));

        //set up writer
        writeAuto writer = new writeAuto(LOG_NAME);


        //initialize hardware
        pitchLeft = hardwareMap.get(ServoImplEx.class, "pitchLeft");
        //pitchRight = hardwareMap.get(ServoImplEx.class, "pitchRight");

        leftRoller = hardwareMap.get(CRServoImplEx.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServoImplEx.class, "rightRoller");
        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        //autoClaw = new autoClaw(pitch, roll, claw);
        wrist = new Wrist(pitchLeft);
        //clawCode = new Claw(claw);
        autoBigWheelIntake bigWheelIntake = new autoBigWheelIntake(leftRoller, rightRoller);
        color = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor sensor = new org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor(color);


        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "linear actuator switch");
        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowSwitch = hardwareMap.get(RevTouchSensor.class, "elbow switch");
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setTargetPositionTolerance(10);
        rightSlide.setTargetPositionTolerance(10);
        slideSwitch = hardwareMap.get(RevTouchSensor.class, "slide switch");
        slide = new Slide(leftSlide,rightSlide, slideSwitch, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2d(0,0,0));
        elbow = new Elbow(elbowMotor, elbowSwitch, 2500, telemetry);
        homingAgent = new Homing(leftSlide, rightSlide, elbowMotor, linearActuatorMotor, this, telemetry, slideSwitch, actuatorSwitch, elbowSwitch);

        boolean moveElUp = false;
        boolean home = true;
        String telemetryMessage = "ELBOW DOES NOT GO UP 30 DEGREES";
        //wait for user input to begin homing
        while (!gamepad1.a && !isStopRequested()) {
            pinpoint.update();
            telemetry.addLine("Step 1: Check Pinpoint Location. Try Moving Robot");
            telemetry.addLine("Step 2: Choose Homing Type.");
            telemetry.addLine("Hold D-Pad Up and A at the same time to home up.");
            telemetry.addLine("Simply hold A at the same time to home down.");
            telemetry.addData("Position X", pinpoint.getPositionRR().position.x);
            telemetry.addData("Position Y", pinpoint.getPositionRR().position.y);
            telemetry.addData("Position Heading", Math.toDegrees(pinpoint.getPositionRR().heading.toDouble()));

            if (gamepad1.dpad_up){
                telemetryMessage = "ELBOW UP 30 DEGREES, THEN HOME DOWN";
                moveElUp = true;
            }
            if (gamepad1.dpad_down){
                telemetryMessage = "ELBOW DOES NOT GO UP 30 DEGREES";
                moveElUp = false;
            }
            if (gamepad1.dpad_left) {
                telemetryMessage = "DOES NOT HOME";
                home = false;
            }

            telemetry.addLine("You will be homing in this way: " + telemetryMessage);
            telemetry.update();
        }


        //HOMING
        wrist.presetPositionPitch(0.5);
        sleep(250);
        if (home) {
            if (moveElUp) {
                homingAgent.moveElbowUpAndHomeDown();
            } else {
                homingAgent.homeDown();
            }
        }
        /* pitch.setPosition(0.66);

        while (!slide.isHomingSwitchPressed() && !isStopRequested()){
            slide.setSlidePower(-0.2);
            telemetry.addData("slide switch state", slide.isHomingSwitchPressed());
            telemetry.addData("Elbow Angle", elbow.getElbowAngle());
            telemetry.update();
        }
        slide.setSlidePower(0);


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

        elbowMotor.setTargetPosition(-100);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(0.2);
        while (elbowMotor.getCurrentPosition() > -99) {

        }
        elbowMotor.setPower(0);

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

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        //wait for user input to begin interpreter parsing and setup
        while(!gamepad1.b && !isStopRequested()) {
            telemetry.addLine("Homing Complete!");
            telemetry.addLine("Add Specimen, then click B to continue.");
            telemetry.addLine("Left Bumper: Intake. Right Bumper: Outtake");
            telemetry.update();
            if (gamepad1.left_bumper) {
                bigWheelIntake.setPower(1);
            } else if (gamepad1.right_bumper) {
                bigWheelIntake.setPower(-1);
            } else {
                bigWheelIntake.setPower(0);
            }
        }


        //initalize pinpoint drive
        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose, telemetry);
        drive.pinpoint.setPosition(beginPose);

        //initialize trajaction builder to parse data
        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);


        //we use these booleans to make sure that we dont add redundant actions to the trajectory
        boolean XareSame = false;
        boolean YareSame = false;
        boolean AngleareSame = false;
        boolean ElbowareSame = false;
        boolean SlideareSame = false;
        boolean PitchareSame = false;
        boolean RollareSame = false;
        boolean ClawareSame = false;
        boolean waitZero = false;
        boolean correctionAreSame = false;

        //build trajectory based on file data
        for (int i = 1; i < vector.size(); i++) {
            XareSame = ((extractAuto.getXFromList(vector.get(i-1)) == extractAuto.getXFromList(vector.get(i))));
            YareSame = ((extractAuto.getYFromList(vector.get(i-1)) == extractAuto.getYFromList(vector.get(i))));
            AngleareSame = ((extractAuto.getAngleFromList(vector.get(i-1)) == extractAuto.getAngleFromList(vector.get(i))));
            PitchareSame = (extractAuto.getPitchFromList(vector.get(i-1)) == extractAuto.getPitchFromList(vector.get(i)));
            RollareSame = (extractAuto.getRollFromList(vector.get(i-1)) == extractAuto.getRollFromList(vector.get(i)));
            ClawareSame = (extractAuto.getClawFromList(vector.get(i-1)) == extractAuto.getClawFromList(vector.get(i)));
            waitZero = extractAuto.getWaitFromList(vector.get(i)) == 0;
            correctionAreSame = (extractAuto.getCorrectionFromList(vector.get(i-1)) == extractAuto.getCorrectionFromList(vector.get(i)));

            if (!correctionAreSame) {
                traj1 = traj1.stopAndAdd(robot.correctionChanger(extractAuto.getCorrectionFromList(vector.get(i))));
            }

            if (XareSame && YareSame && AngleareSame) {
                if (true) {
                    traj1 = traj1.stopAndAdd(elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))));
                }
                if (true) {
                    traj1 = traj1.stopAndAdd(slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i)), extractAuto.getSlideSpeedFromList(vector.get(i))));
                }
            }

            else if (XareSame && YareSame && !AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i)), extractAuto.getSlideSpeedFromList(vector.get(i))))
                        .turnTo(extractAuto.getAngleFromList(vector.get(i)), new TurnConstraints(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01, extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01, extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01));
            }

            else if ((!XareSame || !YareSame) && AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i)), extractAuto.getSlideSpeedFromList(vector.get(i))));
                if (extractAuto.getMoveTypeFromList(vector.get(i)).equals("spline")) {
                    traj1 = traj1.splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)), extractAuto.getAngleFromList(vector.get(i)) ), extractAuto.getTangentFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
                else {
                    traj1 = traj1.strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01), new AngularVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01))));
                }
            }
            else {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i)), extractAuto.getSlideSpeedFromList(vector.get(i))));
                if (extractAuto.getMoveTypeFromList(vector.get(i)).equals("spline")) {
                    traj1 = traj1.splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)), extractAuto.getAngleFromList(vector.get(i)) ), extractAuto.getTangentFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
                else {
                    traj1 = traj1.strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01), new AngularVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01))));
                }
            }
            if (!PitchareSame || !RollareSame || !ClawareSame) {
                //traj1 = traj1.stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))));
                if (extractAuto.getClawFromList(vector.get(i)) == 2) {
                    traj1.stopAndAdd(bigWheelIntake.colorIntake());
                } else {
                traj1 = traj1.stopAndAdd(bigWheelIntake.bigWheelIntakePower(extractAuto.getClawFromList(vector.get(i)))); }
                traj1 = traj1.stopAndAdd(wrist.wristControl(extractAuto.getPitchFromList(vector.get(i))));
                //traj1 = traj1.stopAndAdd(clawCode.clawControl(extractAuto.getClawFromList(vector.get(i))));
            }

            if (!waitZero) {
                traj1 = traj1.waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }

            traj1 = traj1.stopAndAdd(robot.vectorLog(i, telemetry));

            telemetry.addData("Vector " + (i) + " X", extractAuto.getXFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Y", extractAuto.getYFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Heading", extractAuto.getAngleFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Line Type", extractAuto.getMoveTypeFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Tangent", extractAuto.getTangentFromList(vector.get(i)));
            telemetry.addData("Vector " + (i) + " Velocity", extractAuto.getVelocityFromList(vector.get(i)));
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

        //initialize elbow, slide, and claw to starting positions
        elbow.setTargetAngleAndSpeed(ELBOW_START, 1);
        //autoClaw.setPitch(PITCH_START);
        //autoClaw.setClaw(0.21);
        pitchLeft.setPosition(PITCH_START);



        while(!gamepad1.y && !isStopRequested()) {
            telemetry.addLine("Press Y to get into ready-to-run position.");
            telemetry.addData("The elbow angle should be at: ", ELBOW_START);
            telemetry.addData("It is at: ", elbow.getElbowAngle());
            telemetry.update();

        }

        elbowMotor.setPower(0);
        slide.setSlidePower(0);
        pitchLeft.setPwmDisable();


        //autoClaw.setClaw(CLAW_START);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Ready to run! Get out of the field");
        telemetry.update();


        waitForStart();
        timer.reset();

        if (isStopRequested()) {
            return;
        }


        //run trajectory
        Actions.runBlocking(action1);

        //write time it takes to file
        writer.timer(timer.time());
    }
}
