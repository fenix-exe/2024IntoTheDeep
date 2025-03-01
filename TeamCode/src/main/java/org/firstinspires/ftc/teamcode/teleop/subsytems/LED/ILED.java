package org.firstinspires.ftc.teamcode.teleop.subsytems.LED;

public interface ILED {
    public enum LEDColor {WHITE,RED,YELLOW}
    public void setColor(LEDColor color);
    public void turnOff();

}
