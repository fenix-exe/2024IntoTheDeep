package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.Servo;

import java.time.zone.ZoneRules;

public abstract class CommonWrist {

    protected Servo pitch;
    protected Servo roll;
    protected static double PITCH_OFFSET = 3;


    public CommonWrist(Servo pitch, Servo roll) {
        this.pitch = pitch;
        this.roll = roll;
    }

    public CommonWrist(Servo pitch){
        this.pitch = pitch;
    }


    public void presetPositionPitch (double presetPosition){
        pitch.setPosition(presetPosition);
    }
    public double getPitchAngle(){
        return pitch.getPosition();
    }
}
