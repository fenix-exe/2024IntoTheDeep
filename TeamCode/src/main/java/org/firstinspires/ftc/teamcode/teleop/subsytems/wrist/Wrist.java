package org.firstinspires.ftc.teamcode.teleop.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.CommonWrist;

public class Wrist extends CommonWrist {
    Servo pitchLeft;
    private static final double PITCH_OFFSET = 3;

    public Wrist(Servo pitch){
        super(pitch);
        pitchLeft = pitch;
    }
    public Wrist(Servo pitchLeft, Servo pitchRight){
        super(pitchLeft, pitchRight);
        this.pitchLeft = pitchLeft;
    }
    public void manualControlPitch(double stepSize){
        double targetPosition = stepSize + pitchLeft.getPosition();
        if (targetPosition > 1 - CommonWrist.SERVO_OFFSET){
            targetPosition = 1 - CommonWrist.SERVO_OFFSET;
        }
        if (targetPosition < CommonWrist.SERVO_OFFSET){
            targetPosition = CommonWrist.SERVO_OFFSET;
        }
        pitchLeft.setPosition(targetPosition);
    }

}
