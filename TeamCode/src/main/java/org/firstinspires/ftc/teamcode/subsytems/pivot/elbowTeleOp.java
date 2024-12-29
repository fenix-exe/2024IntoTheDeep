package org.firstinspires.ftc.teamcode.subsytems.pivot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class elbowTeleOp extends LinearOpMode {
    DcMotorEx pivot;
    pivotCodeFunctions pivotCode;
    PivotPIDFFunctions pivotPIDF;
    PIDController controllerPivotPIDF;
    public enum pivotPos {MOVING_TO_POSITION , JOYSTICK_CONTROL}
    pivotPos pivotStateMachine;
    @Override
    public void runOpMode() throws InterruptedException {
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controllerPivotPIDF = new PIDController(0.014, 0, 0.0004);
        pivotPIDF = new PivotPIDFFunctions(controllerPivotPIDF, 0);
        pivotCode = new pivotCodeFunctions(pivot, pivotPIDF, 2187);
        pivotStateMachine = pivotPos.JOYSTICK_CONTROL;



        waitForStart();

        while (opModeIsActive()){
            pivotCode.pivotJoystick(pivot.getCurrentPosition(), gamepad2.left_stick_x);
            /*if (gamepad2.b){
                    pivotCode.goTo(0);
                    pivotStateMachine = pivotPos.MOVING_TO_POSITION;
            } else if (gamepad2.y){
                pivotCode.goTo(2187);
                pivotStateMachine = pivotPos.MOVING_TO_POSITION;
            } else if (gamepad2.left_stick_x == 0 && pivotStateMachine != pivotPos.MOVING_TO_POSITION){
                pivotCode.goTo(pivot.getCurrentPosition());
            } else if (gamepad2.left_stick_x < 0){
                    pivotCode.pivotJoystick(pivot.getCurrentPosition(), gamepad2.left_stick_x);
                    pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
            } else if (gamepad2.left_stick_x > 0) {
                pivotCode.pivotJoystick(pivot.getCurrentPosition(), gamepad2.left_stick_x);
                pivotStateMachine = pivotPos.JOYSTICK_CONTROL;
            }*/
        }
    }
}
