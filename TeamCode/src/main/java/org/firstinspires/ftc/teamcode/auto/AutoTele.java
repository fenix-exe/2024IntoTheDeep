package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import page.j5155.expressway.ftc.actions.ActionLinearOpMode;
import page.j5155.expressway.ftc.actions.ActionOpMode;
import page.j5155.expressway.ftc.actions.ActionRunner;
import page.j5155.expressway.ftc.motion.PIDToPoint;

import static page.j5155.expressway.core.geometry.GeometryHelpers.distanceTo;

@TeleOp
public class AutoTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        Pose2d target = new Pose2d(24, 0, Math.toRadians(0));
        Pose2d home = new Pose2d(0,0,0);
        MecanumDrive.PIDDrive targetMove = drive.pidToPointAction(target, telemetry);
        MecanumDrive.PIDDrive homeMove = drive.pidToPointAction(home, telemetry);
        TelemetryPacket p = new TelemetryPacket();
        ActionRunner runner = new ActionRunner();
        float lateral;
        double strafe;
        float yaw;
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            //Actions.runBlocking(targetMove);
            if (gamepad1.a) {
                runner.runAsync(targetMove);
                runner.updateAsync();
                telemetry.addData("pin x",drive.pose.position.x);
                telemetry.addData("pin y",drive.pose.position.y);
                telemetry.addData("pin h",Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();
            } else {
                runner.getRunningActions().clear();
                lateral = gamepad1.left_stick_y * -1;
                strafe = gamepad1.left_stick_x * 1.1;
                yaw = gamepad1.right_stick_x;
                double denominator = Math.max(1, Math.abs(lateral+strafe+yaw));
                FL.setPower(((lateral + strafe + yaw) / denominator));
                BL.setPower((((lateral - strafe) + yaw) / denominator));
                FR.setPower((((lateral - strafe) - yaw) / denominator));
                BR.setPower((((lateral + strafe) - yaw) / denominator));
                telemetry.addData("pin x",drive.pose.position.x);
                telemetry.addData("pin y",drive.pose.position.y);
                telemetry.addData("pin h",Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();
            }
            telemetry.addData("pin x",drive.pose.position.x);
            telemetry.addData("pin y",drive.pose.position.y);
            telemetry.addData("pin h",Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

        }
    }
}















































































































