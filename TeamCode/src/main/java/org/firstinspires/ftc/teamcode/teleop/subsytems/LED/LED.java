package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LED {
    RevBlinkinLedDriver LEDForRobot;
    public LED (RevBlinkinLedDriver LED){
        this.LEDForRobot = LED;
    }
    public void setColorEndgame(){
        LEDForRobot.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }
}
