package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.Servo;

import java.time.zone.ZoneRules;

public abstract class CommonWrist {
    Servo pitch;

    public CommonWrist(Servo pitch) {
        this.pitch = pitch;
    }
    public void presetPositionPitch (double presetPosition){
        pitch.setPosition(presetPosition);
    }
    public double getPitchAngle(){
        return pitch.getPosition();
    }
}
