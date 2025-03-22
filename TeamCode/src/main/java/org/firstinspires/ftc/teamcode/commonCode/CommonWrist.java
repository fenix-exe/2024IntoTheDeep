package org.firstinspires.ftc.teamcode.commonCode;

import com.qualcomm.robotcore.hardware.Servo;

public abstract class CommonWrist {
    Servo pitch;
    Servo roll;
    protected static double PITCH_OFFSET = 3;

    public CommonWrist(Servo pitch, Servo roll){
        this.pitch = pitch;
        this.roll = roll;
    }
    public void presetPositionPitch (double presetPosition){
        pitch.setPosition((presetPosition-PITCH_OFFSET)/180 + 0.5);
    }
    public void presetPositionRoll (double presetPosition){
        roll.setPosition(presetPosition/300 + 0.5);
    }
    public void presetPosition(double pitch, double roll){
        //divide by 300 to convert angles to servo positions for pitch
        //0.5 is the middle position of the servo, maximum of +150 to -150 degrees
        //divide by 180 to convert angles to servo positions for roll
        //0.5 is the middle position of the servo, maximum of -90 to 90 degrees
        presetPositionPitch(pitch);
        presetPositionRoll(roll);
    }
    public double getRollAngle(){
        return 300 * roll.getPosition() - 150 + PITCH_OFFSET;
    }
    public double getPitchAngle(){
        return 180 * pitch.getPosition() - 90;
    }
}
