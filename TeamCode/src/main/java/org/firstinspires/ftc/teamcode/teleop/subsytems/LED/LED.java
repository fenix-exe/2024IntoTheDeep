package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LED implements ILED{
    RevBlinkinLedDriver LEDForRobot;
    public LED (RevBlinkinLedDriver LED){
        this.LEDForRobot = LED;
    }


    @Override
    public void setColor(LEDColor color) {
        RevBlinkinLedDriver.BlinkinPattern pattern;
        switch (color){
            case RED:
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                break;
            case BLUE:
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                break;
            default:
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }
        LEDForRobot.setPattern(pattern);
    }
}
