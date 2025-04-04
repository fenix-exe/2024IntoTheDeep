package org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    private final RevColorSensorV3 colorSensor;
    double H;
    double S;
    double V;
    double colorSensorDistance;
    boolean detectColor;
    double colorSensorDetectionDistance = 8;
    public ColorSensor(RevColorSensorV3 colorSensor){
        this.colorSensor =colorSensor;
    }
    public void updateHSVandDistance(){
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        H = JavaUtil.rgbToHue(red, green, blue);
        S = JavaUtil.rgbToSaturation(red, green, blue);
        V = JavaUtil.rgbToValue(red, green, blue);
        colorSensorDistance = colorSensor.getDistance(DistanceUnit.MM);
    }
    public void updateDetectColor(){
        detectColor = colorSensorDistance < colorSensorDetectionDistance;
    }

    /**
     * This function returns the distance from the color sensor in millimeters
     * @return distance in millimeters
     */
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
    public boolean isConnected(){
        return colorSensor.getDeviceID() == -62;
    }

}
