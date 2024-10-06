package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "pivotCode")
public class pivotCode extends LinearOpMode {
    private DcMotorEx pivot;
    private DcMotorEx slide;
    private int pivotPos;
    PivotPIDFFunctions controller;
    @Override
    public void runOpMode() throws InterruptedException {
        /* dead weight PID
        controller = new PivotPIDFFunctions(new PIDController(0.005, 0, 0.0005), 0);
         */
        //PID with motor and 7 hole U-channel
        controller = new PivotPIDFFunctions(new PIDController(0.02, 0 ,0.0005),0);
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide =hardwareMap.get(DcMotorEx.class, "slide");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            if (slide.getCurrentPosition() < 4000){
                controller.setPID(0.02,0.0005);
            } else {
                controller.setPID(0.05,0.0008);
            }
            pivot.setPower(controller.moveToPos(pivot.getCurrentPosition(), pivotPos));
            //get power from PID
            telemetry.addData("pivotPos", pivotPos);
        }
    }
}