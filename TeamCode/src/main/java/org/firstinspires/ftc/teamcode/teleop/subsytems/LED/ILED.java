package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

public interface ILED {
    public enum LEDColor {WHITE,RED, BLUE,GREEN,ORANGE,PURE_RED,PURE_BLUE,YELLOW,PURE_GREEN}
    public void setColor(LEDColor color);
    public void turnOff();

}
