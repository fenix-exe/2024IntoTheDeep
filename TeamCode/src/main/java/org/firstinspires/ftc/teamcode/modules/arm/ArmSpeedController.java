package org.firstinspires.ftc.teamcode.modules.arm;

import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;

public class ArmSpeedController {
    public static Slide slide;

    public static double getElbowPowerLimit(){
        //TODO tune
        return 1 - slide.getSlideExtensionInInches()/60;
    }
}
