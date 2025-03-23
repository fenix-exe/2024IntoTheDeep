package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;

public class Color {
    private final RevColorSensorV3 colorSensor;
    public Color(RevColorSensorV3 colorSensor){
        this.colorSensor =colorSensor;
    }
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.MM);
    }
    public double getRed(){
        return colorSensor.red();
    }
    public double getBlue(){
        return colorSensor.blue();
    }
    public double getGreen(){
        return colorSensor.green();
    }
    public double getAlpha(){
        return colorSensor.alpha();
    }
}
