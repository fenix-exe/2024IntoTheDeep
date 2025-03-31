package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public abstract class CommonWrist {

    protected Servo pitchLeft;
    public static double SERVO_OFFSET = 0.005;
    public CommonWrist(Servo pitch){
        this.pitchLeft = pitch;
    }
    public CommonWrist(Servo pitchLeft, Servo pitchRight){
        this.pitchLeft = pitchLeft;
    }


    public void presetPositionPitch (double presetPosition){
        pitchLeft.setPosition(presetPosition);
    }
    public double getPitchAngle(){
        return pitchLeft.getPosition();
    }
}
