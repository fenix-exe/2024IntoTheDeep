package org.firstinspires.ftc.teamcode.util.testCode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class hallTest extends LinearOpMode {
    RevTouchSensor touch;
    @Override
    public void runOpMode() throws InterruptedException {
        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}
