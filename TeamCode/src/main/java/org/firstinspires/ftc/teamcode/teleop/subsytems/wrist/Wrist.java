package org.firstinspires.ftc.teamcode.teleop.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.CommonWrist;

public class Wrist extends CommonWrist {
    Servo pitchServo;
    private static final double PITCH_OFFSET = 3;

    public Wrist(Servo pitch){
        super(pitch);
        pitchServo = pitch;
    }
    public void manualControlPitch(double stepSize){
        double targetPosition = stepSize + pitchServo.getPosition();
        if (targetPosition > 1){
            targetPosition = 1;
        }
        if (targetPosition < 0){
            targetPosition = 0;
        }
        pitchServo.setPosition(targetPosition);
    }

}
