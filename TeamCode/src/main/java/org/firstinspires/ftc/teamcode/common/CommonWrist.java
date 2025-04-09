package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.extractOffsets;

import java.io.IOException;

@Config
public abstract class CommonWrist {

    protected Servo pitchLeft;
    private extractOffsets offsets = new extractOffsets();
    public static double SERVO_OFFSET = 0;
    public CommonWrist(Servo pitch){
        this.pitchLeft = pitch;
        try {
            offsets.offsetGetter("sdcard/Download/offsets/clipOffsets.csv");
        } catch (IOException e) {
            SERVO_OFFSET = 0;
        }
        SERVO_OFFSET = offsets.getPitchOffset();
    }

    public CommonWrist(Servo pitchLeft, Servo pitchRight){
        this.pitchLeft = pitchLeft;
    }


    public void presetPositionPitch (double presetPosition){
        pitchLeft.setPosition(presetPosition + SERVO_OFFSET);
    }
    public double getPitchAngle(){
        return pitchLeft.getPosition();
    }
}
