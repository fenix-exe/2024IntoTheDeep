package org.firstinspires.ftc.teamcode.subsytems.elbow.test;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
//@TeleOp
public class HallEffectSensorTest extends LinearOpMode {
    RevTouchSensor limitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Is the limit switch activated?", limitSwitch.isPressed());
            telemetry.update();
        }
    }
}
