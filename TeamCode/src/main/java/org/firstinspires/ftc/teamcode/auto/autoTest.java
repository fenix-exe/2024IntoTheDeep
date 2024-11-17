package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;
import org.firstinspires.ftc.teamcode.subsytems.pivot.PivotPIDFFunctions;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;
import org.firstinspires.ftc.teamcode.util.writeAuto;

import java.util.WeakHashMap;

@Autonomous
public class autoTest extends LinearOpMode {

    slideCodeFunctions slideCode;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    DcMotorEx slide;
    DcMotorEx pivot;
    writeAuto writer;
    @Override
    public void runOpMode() throws InterruptedException {

        /*slide = hardwareMap.get(DcMotorEx.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);*/
        String filename = "/sdcard/Download/test.csv";
        writer = new writeAuto(filename);


        //Homing the pivot
        /*while (!limitSwitch.isPressed()){
            pivot.setPower(-0.5);
        }
        pivot.setPower(0);*/
        /*pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideCode = new slideCodeFunctions(slide);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, 2178);*/

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.a) {
                writer.writeToFile(1, 2, 3, 4);
            }
            if (gamepad1.b) {
                writer.writeToFile(10, 20, 30, 40);
            }
        }



    }
}
