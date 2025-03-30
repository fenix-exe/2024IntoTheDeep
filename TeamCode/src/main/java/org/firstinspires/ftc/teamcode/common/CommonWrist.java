package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public abstract class CommonWrist {

    protected Servo pitchLeft;
    protected Servo pitchRight;
    public static double SERVO_OFFSET = 0.005;


    public CommonWrist(Servo pitchLeft, Servo pitchRight) {
        this.pitchLeft = pitchLeft;
        this.pitchRight = pitchRight;
    }

    public CommonWrist(Servo pitch){
        this.pitchLeft = pitch;
    }


    public void presetPositionPitch (double presetPosition){
        pitchLeft.setPosition(presetPosition);
        pitchRight.setPosition(1-pitchLeft.getPosition()-SERVO_OFFSET);
    }
    public double getPitchAngle(){
        return pitchLeft.getPosition();
    }
}
