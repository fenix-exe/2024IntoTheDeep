package org.firstinspires.ftc.teamcode.util.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
@Config
@TeleOp
public class diffTELEOP extends LinearOpMode {
    ServoImplEx pitch;
    ServoImplEx claw;
    ServoImplEx roll;
    DcMotor slide;
    DcMotor pivot;
    public static double clawnum = 0;
    public static double pitchnum = 0;
    public static double rollnum = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        pitch = hardwareMap.get(ServoImplEx.class,"pitch");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        roll = hardwareMap.get(ServoImplEx.class, "roll");
        slide = hardwareMap.get(DcMotor.class, "slide");
        pivot = hardwareMap.get(DcMotor.class, "pivot");



        waitForStart();
        while (opModeIsActive()) {

            pitch.setPosition(pitchnum);
            claw.setPosition(clawnum);
            roll.setPosition(rollnum);

            slide.setPower(gamepad1.left_stick_y*0.3);
            pivot.setPower(gamepad1.left_stick_x*0.3);


            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("pitch", pitch.getPosition());

            telemetry.update();
        }
    }
}
