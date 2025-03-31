package org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorSensorStatus extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Color Sensor Status", colorSensorV3.status());
            telemetry.update();
        }
    }
}
