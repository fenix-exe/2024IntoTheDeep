package org.firstinspires.ftc.teamcode.subsytems.activeIntake.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ActiveIntakeSensorTeleOp extends LinearOpMode {
    RevColorSensorV3 colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "active intake sensor");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("distance in mm", colorSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("red hue", colorSensor.red());
            telemetry.addData("blue hue", colorSensor.blue());
            telemetry.addData("green hue", colorSensor.green());
            telemetry.update();

        }
    }
}
