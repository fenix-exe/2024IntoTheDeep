package org.firstinspires.ftc.teamcode.subsytems.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PIDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx elbow = hardwareMap.get(DcMotorEx.class, "pivot");
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDControl controller = new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0, 24.2);
        int targetPosition = 0;
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                targetPosition = 0;

            }
            if (gamepad1.b){
                targetPosition = 1000;
            }
            if (gamepad1.x){
                targetPosition = 2000;
            }
            elbow.setPower(controller.moveToPosition(elbow.getCurrentPosition(), targetPosition));
            telemetry.addData("Elbow Power", elbow.getPower());
            telemetry.addData("Elbow Position", elbow.getCurrentPosition());
            telemetry.update();
        }
    }
}
