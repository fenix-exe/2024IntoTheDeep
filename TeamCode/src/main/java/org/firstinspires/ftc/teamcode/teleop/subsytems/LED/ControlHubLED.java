package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class ControlHubLED implements ILED{
    LynxModule led;
    ArrayList<Blinker.Step> steps;
    public ControlHubLED(LynxModule led){
        this.led = led;
    }
    @Override
    public void setColor(LEDColor color) {
        steps = new ArrayList<Blinker.Step>();
        steps.add(new Blinker.Step(convertToLynxColor(color), 500, TimeUnit.MILLISECONDS));
        led.setPattern(steps);
    }
    private int convertToLynxColor(LEDColor color){
        switch (color){
            case BLUE:
                return 10;
            case RED:
                return 20;
            case GREEN:
                return 30;
            default:
                return 0;
        }
    }
}
