package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "pivotCode")
public class slideCode extends LinearOpMode {
    private DcMotorEx pivot;
    private int pivotPos;
    PivotPIDFFunctions controller;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PivotPIDFFunctions(new PIDController(0.005, 0, 0.0005), 0);
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x) {
                pivotPos = 0;
                // Down Position
            } else if (gamepad1.y) {
                pivotPos = -2400;
                // Up Position
            }
            if (pivotPos < -2400) {
                pivotPos = -2400;
                //Upward limit
            }
            if (pivotPos > -20) {
                pivotPos = -20;
                //Downward limit
            }
            pivot.setPower(controller.moveToPos(pivot.getCurrentPosition(), pivotPos));
            //get power from PID
            telemetry.addData("pivotPos", pivotPos);
        }
    }
}