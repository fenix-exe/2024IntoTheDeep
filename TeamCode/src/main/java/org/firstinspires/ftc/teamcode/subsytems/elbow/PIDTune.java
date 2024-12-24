package org.firstinspires.ftc.teamcode.subsytems.elbow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Config
public class PIDTune extends LinearOpMode {
    private PIDController controller;
    public static double p=0,i=0,d=0;
    public static double f=0;
    public static int target = 0;
    private final double ticks_in_degrees = 8719.2/360;
    private DcMotorEx elbow;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elbow = hardwareMap.get(DcMotorEx.class, "pivot");
        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p, i ,d);
            int armPos = elbow.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees))*f;
            double power = pid +ff;
            elbow.setPower(power);
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}