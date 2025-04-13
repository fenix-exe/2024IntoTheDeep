package org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class ColorSensorStatus extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        waitForStart();
        while (opModeIsActive()){
            int argb = colorSensorV3.argb();
            telemetry.addData("Color Sensor Status", colorSensorV3.status());
            telemetry.addData("Color Sensor Device ID", colorSensorV3.getDeviceID());
            telemetry.addData("Color Sensor Connection Info", colorSensorV3.getConnectionInfo());
            telemetry.addData("Pure ARGB Values", argb);
            telemetry.addData("Bitshifted ARGB red value", (argb >> 16) & 0xFF);
            telemetry.addData("Bitshifted ARGB blue value", (argb >> 0) & 0xFF);
            telemetry.addData("Bitshifted ARGB green value",  (argb >> 8) & 0xFF);
            telemetry.addData("color sensor red not from ARGB", colorSensorV3.red());
            telemetry.addData("color sensor blue not from ARGB", colorSensorV3.blue());
            telemetry.addData("color sensor green not from ARGB", colorSensorV3.green());
            telemetry.update();
        }
    }
}
