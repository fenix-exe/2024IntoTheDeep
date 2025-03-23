package org.firstinspires.ftc.teamcode.teleop.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.CommonWrist;

public class Wrist extends CommonWrist {
    Servo pitchServo;
    Servo rollServo;
    private static double PITCH_OFFSET = 3;

    public Wrist(Servo pitch, Servo roll){
        super(pitch, roll);
        pitchServo = pitch;
        rollServo = roll;
    }
    public void manualControlPitch(double stepSizeInDegrees){
        double targetPosition = stepSizeInDegrees/180 + pitchServo.getPosition();
        if (targetPosition > 1){
            targetPosition = 1;
        }
        if (targetPosition < 0){
            targetPosition = 0;
        }
        pitchServo.setPosition(targetPosition);
    }
    public void manualControlRoll(double stepSizeInDegrees){
        //step size is divided because it is in angles, not servo position
        double targetPosition = stepSizeInDegrees/300 + rollServo.getPosition();
        if (targetPosition > 1){
            targetPosition = 1;
        }
        if (targetPosition < 0){
            targetPosition = 0;
        }
        rollServo.setPosition(targetPosition);
    }

}
