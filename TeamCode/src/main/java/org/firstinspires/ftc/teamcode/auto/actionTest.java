package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

@Autonomous
public class actionTest extends LinearOpMode {
    CRServo intake;
    ServoImplEx left;
    ServoImplEx right;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0,0,180);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = hardwareMap.get(CRServo.class, "intake");
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        activeIntake activeIntake = new activeIntake(intake);
        differential diffy = new differential(left, right);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(new Pose2d(0,0,0));
        Action action1 = traj1.afterTime(3,(diffy.setDiffy(-45,90))).build();




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
