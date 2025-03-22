package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.auto.subsystems.claw.autoClaw;
import org.firstinspires.ftc.teamcode.auto.subsystems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.auto.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.auto.util.RobotWideFunctions;
import org.firstinspires.ftc.teamcode.auto.util.extractAuto;
import org.firstinspires.ftc.teamcode.auto.util.writeAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;


@Autonomous(name = "AUTO - BUCKET 4!!!!", preselectTeleOp = "TeleOpV5SampleZapdos")
public class ascentPreloadPark extends LinearOpMode {

    //initialize auto extractor
    String FILE_NAME = "/sdcard/Download/autoPositions/ascentPreloadPark.csv";
    int ELBOW_START = 7;
    int SLIDE_START = 0;
    double PITCH_START = 1;
    double ROLL_START = 0.15;
    double CLAW_START = 0.86;


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

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    Slide slide;
    RevTouchSensor homingSwitch;
    public GoBildaPinpointDriverRR pinpoint;


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

        writeAuto writer = new writeAuto("ascenPreloadParkTime");

        //set up rr


        pitch = hardwareMap.get(ServoImplEx.class, "pitch");
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



        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "homing switch");
        slide = new Slide(leftSlide,rightSlide, homingSwitch);

        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2d(0,0,0));
        elbow = new Elbow(elbowMotor, limitSwitch, 2500);


        while (!gamepad1.a && !isStopRequested()) {
            pinpoint.update();
            telemetry.addData("pose x", pinpoint.getPositionRR().position.x);
            telemetry.addData("pose y", pinpoint.getPositionRR().position.y);
            telemetry.addData("pose head", Math.toDegrees(pinpoint.getPositionRR().heading.toDouble()));
            telemetry.update();
        }

        //HOMING
        pitch.setPosition(1);

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
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!gamepad1.b && !isStopRequested()) {

        }

        Pose2d beginPose = new Pose2d(extractAuto.getXFromList(vector.get(0)), extractAuto.getYFromList(vector.get(0)), extractAuto.getAngleFromList(vector.get(0)));

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        drive.pinpoint.setPosition(beginPose);


        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose);

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
                if (!ElbowareSame) {
                    traj1 = traj1.stopAndAdd(elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))));
                }
                if (!SlideareSame) {
                    traj1 = traj1.stopAndAdd(slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))));
                }
            }
            else if (XareSame && YareSame && !AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))))
                        .turnTo(extractAuto.getAngleFromList(vector.get(i)), new TurnConstraints(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01, extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01, extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxAngVel*0.01));
            }
            else if ((!XareSame || !YareSame) && AngleareSame) {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))));
                if (extractAuto.getMoveTypeFromList(vector.get(i)).equals("spline")) {
                    traj1 = traj1.splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)), extractAuto.getAngleFromList(vector.get(i)) ), extractAuto.getTangentFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
                else {
                    traj1 = traj1.strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
            }
            else {
                traj1 = traj1
                        .afterDisp(0,elbow.elbowControl(extractAuto.getElbowPhiFromList(vector.get(i)), extractAuto.getElbowSpeedFromList(vector.get(i))))
                        .afterDisp(0,slide.slideControl(extractAuto.getLinearSlideFromList(vector.get(i))));
                if (extractAuto.getMoveTypeFromList(vector.get(i)).equals("spline")) {
                    traj1 = traj1.splineToLinearHeading(new Pose2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)), extractAuto.getAngleFromList(vector.get(i)) ), extractAuto.getTangentFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
                else {
                    traj1 = traj1.strafeToLinearHeading(new Vector2d(extractAuto.getXFromList(vector.get(i)),extractAuto.getYFromList(vector.get(i)) ), extractAuto.getAngleFromList(vector.get(i)), new TranslationalVelConstraint(extractAuto.getVelocityFromList(vector.get(i))*MecanumDrive.PARAMS.maxWheelVel*0.01));
                }
            }
            if (!PitchareSame || !RollareSame || !ClawareSame) {
                traj1 = traj1.stopAndAdd(autoClaw.clawControl(extractAuto.getPitchFromList(vector.get(i)),extractAuto.getRollFromList(vector.get(i)), extractAuto.getClawFromList(vector.get(i))));
            }

            if (!waitZero) {
                traj1 = traj1.waitSeconds(extractAuto.getWaitFromList(vector.get(i)));
            }

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

        elbow.setTargetAngleAndSpeed(ELBOW_START, 1);
        autoClaw.setPitch(PITCH_START);
        autoClaw.setRoll(ROLL_START);
        autoClaw.setClaw(0.21);


        if (elbow.degreesToTicks(ELBOW_START)-30 < elbowMotor.getCurrentPosition() && elbowMotor.getCurrentPosition() < elbow.degreesToTicks(ELBOW_START)+30) {
            elbowMotor.setPower(0);

        } else {
            elbow.setTargetAngleAndSpeed(ELBOW_START, 1);
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
        pitch.setPwmDisable();
    }
}
