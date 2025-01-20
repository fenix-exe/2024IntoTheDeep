package org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.test;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
@TeleOp
public class LinearActuatorTest extends LinearOpMode {
    DcMotorEx linearActuatorMotor;
    LinearActuator linearActuator;
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad gp1 = new Gamepad();
        gp1.copy(gamepad1);
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class, "linear actuator");
        RevTouchSensor actuatorSwitch = hardwareMap.get(RevTouchSensor.class, "actuator switch");

        linearActuator = new LinearActuator(linearActuatorMotor, actuatorSwitch);

        linearActuator.goToTargetPositionInches(0);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                linearActuator.goToTargetPositionInches(0);
            } else if (gamepad1.b){
                linearActuator.goToTargetPositionInches(9.5);
            } else if (gamepad1.x){
                linearActuator.goToTargetPositionInches(6);
            } else if (Math.abs(gamepad1.right_stick_y) > 0){
                linearActuator.goToTargetPositionInches(linearActuator.ticksToInches(linearActuatorMotor.getCurrentPosition()) - 3*gamepad1.right_stick_y);
            }
            gp1.copy(gamepad1);
        }
    }
}
