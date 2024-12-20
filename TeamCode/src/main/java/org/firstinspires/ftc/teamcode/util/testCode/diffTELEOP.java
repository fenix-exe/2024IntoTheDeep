package org.firstinspires.ftc.teamcode.util.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.differential.Differential;
@Config
@TeleOp
public class diffTELEOP extends LinearOpMode {
    ServoImplEx left;
    ServoImplEx right;
    Differential diffCode;
    CRServo intake;
    DriverControls controls;
    public static double pitch = 0;
    public static double roll = 0;
    public static double intPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(ServoImplEx.class,"left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        intake = hardwareMap.get(CRServo.class, "intake");
        diffCode = new Differential(left, right);



        waitForStart();
        while (opModeIsActive()) {

            diffCode.setDifferentialPosition(pitch, roll);
            intake.setPower(intPower);


            telemetry.addData("loca", left.getPosition());
            telemetry.addData("locb", right.getPosition());

            telemetry.update();
        }
    }
}
