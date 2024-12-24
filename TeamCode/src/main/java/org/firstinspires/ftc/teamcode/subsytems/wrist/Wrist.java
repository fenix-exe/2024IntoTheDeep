package org.firstinspires.ftc.teamcode.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    Servo pitch;
    Servo roll;

    public Wrist(Servo pitch, Servo roll){
        this.pitch = pitch;
        this.roll = roll;
    }
    public void manualControlPitch(double stepSizeInDegrees){
        double targetPosition = stepSizeInDegrees/300 + pitch.getPosition();
        if (targetPosition > 1){
            targetPosition = 1;
        }
        if (targetPosition < 0){
            targetPosition = 0;
        }
        pitch.setPosition(targetPosition);
    }
    public void manualControlRoll(double stepSizeInDegrees){
        //step size is divided because it is in angles, not servo position
        double targetPosition = stepSizeInDegrees/300 + roll.getPosition();
        if (targetPosition > 1){
            targetPosition = 1;
        }
        if (targetPosition < 0){
            targetPosition = 0;
        }
        roll.setPosition(targetPosition);
    }
    public void presetPositionPitch (double presetPosition){
        pitch.setPosition(presetPosition);
    }
    public void presetPositionRoll (double presetPosition){
        roll.setPosition(presetPosition);
    }
    public void presetPosition(double pitch, double roll){
        //divide by 300 to convert angles to servo positions for gobilda servos
        //0.5 is the middle position of the servo, maximum of +150 to -150 degrees
        presetPositionPitch(pitch/300 + 0.5);
        presetPositionRoll(roll/300 + 0.5);
    }

}
