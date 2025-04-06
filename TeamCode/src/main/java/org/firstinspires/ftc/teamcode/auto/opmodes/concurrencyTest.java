package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.auto.subsystems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.auto.subsystems.slide.Slide;

@Autonomous
public class concurrencyTest extends LinearOpMode {

    Elbow elbow;
    DcMotorEx elbowMotor;
    RevTouchSensor elbowSwitch;

    //set up slides
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    Slide slide;
    RevTouchSensor slideSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
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
        slide = new Slide(leftSlide,rightSlide, slideSwitch);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        TrajectoryActionBuilder traj1 = drive.actionBuilder(new Pose2d(0,0,0));
        traj1 = traj1.stopAndAdd(new ParallelAction(slide.slideControl(5, 0.5), elbow.elbowControl(30, 0.5)));
        Action action1 = traj1.build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(action1);
    }
}
