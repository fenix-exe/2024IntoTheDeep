package org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    RevColorSensorV3 colorSensor;

    public ColorSensor(RevColorSensorV3 colorSensor){
        this.colorSensor = colorSensor;
    }
    public double getDistance(DistanceUnit unit){
        return colorSensor.getDistance(unit);
    }
}
