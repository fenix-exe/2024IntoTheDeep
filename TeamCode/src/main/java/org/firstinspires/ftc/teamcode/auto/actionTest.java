package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

@Autonomous
public class actionTest extends LinearOpMode {
    CRServo intake;
    Servo left;
    Servo right;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0,0,180);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = hardwareMap.get(CRServo.class, "intake");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        activeIntake activeIntake = new activeIntake(intake);
        //differential diffy = new differential(left, right);

        waitForStart();

        /*Actions.runBlocking(
                new SequentialAction(activeIntake.aIControl(1),
                        diffy.setDiffy(90, 72))
                );*/

    }
}
