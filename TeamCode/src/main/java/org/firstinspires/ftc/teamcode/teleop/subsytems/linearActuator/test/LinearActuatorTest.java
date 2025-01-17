package org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;

public class LinearActuatorTest extends LinearOpMode {
    private DcMotorEx linearActuatorMotor;
    private LinearActuator linearActuator;
    @Override
    public void runOpMode() throws InterruptedException {
        linearActuatorMotor = hardwareMap.get(DcMotorEx.class,"linear actuator");
        linearActuator = new LinearActuator(linearActuatorMotor);
        linearActuator.resetEncoders();
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                linearActuator.goToTargetPositionInches(0);
            }
            if (gamepad1.b){
                linearActuator.goToTargetPositionInches(6);
            }
            if (gamepad1.x){
                linearActuator.goToTargetPositionInches(5);
            }
        }
    }
}
