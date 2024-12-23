package org.firstinspires.ftc.teamcode.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    Servo pitch;
    Servo roll;

    public Wrist(Servo pitch, Servo roll){
        this.pitch = pitch;
        this.roll = roll;
    }
    public void manualControlPitch(double stepSize, double initPosition){
        double targetPosition = stepSize + initPosition;
        pitch.setPosition(targetPosition);
    }
    public void manualControlRoll(double stepSize, double initPosition){
        double targetPosition = stepSize + initPosition;
        roll.setPosition(targetPosition);
    }
    public void presetPositionPitch (double presetPosition){
        pitch.setPosition(presetPosition);
    }
    public void presetPositionRoll (double presetPosition){
        roll.setPosition(presetPosition);
    }

}
