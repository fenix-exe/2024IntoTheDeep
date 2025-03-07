package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LED implements ILED{
    RevBlinkinLedDriver LEDForRobot;
    boolean isOn;
    public LED (RevBlinkinLedDriver LED){
        this.LEDForRobot = LED;
        isOn = false;
    }


    @Override
    public void setColor(LEDColor color) {
        RevBlinkinLedDriver.BlinkinPattern pattern;
        switch (color){
            case RED:
                pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST;
                break;
            case WHITE:
                pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
                break;
            case GREEN:
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                break;
            case YELLOW:
                pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
                break;
            default:
                pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        }
        LEDForRobot.setPattern(pattern);
        isOn = true;
    }

    @Override
    public void turnOff() {
        LEDForRobot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    public boolean isOn(){
        return isOn;
    }
}
