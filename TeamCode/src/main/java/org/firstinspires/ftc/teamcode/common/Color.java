package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;

public class Color {
    private final RevColorSensorV3 colorSensor;
    double H;
    double S;
    double V;
    double colorSensorDistance;
    boolean detectColor;
    double colorSensorDetectionDistance = 8;
    public Color(RevColorSensorV3 colorSensor){
        this.colorSensor =colorSensor;
    }
    public void updateHSVandDistance(){
        H = JavaUtil.rgbToHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        S = JavaUtil.rgbToSaturation(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        V = JavaUtil.rgbToValue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        colorSensorDistance = colorSensor.getDistance(DistanceUnit.MM);
    }
    public void updateDetectColor(){
        detectColor = colorSensorDistance < colorSensorDetectionDistance;
    }
    public double getDistance(){
        return colorSensorDistance;
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
    public boolean detectingBlue(){
        return H > 220 && H < 230 && detectColor;
    }
    public boolean detectingRed(){
        return H > 10 && H < 20 && detectColor;
    }
    public boolean detectingYellow(){
        return H > 75 && H < 95 && detectColor;
    }
}
