package org.firstinspires.ftc.teamcode.teleop.subsytems.wrist;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.CommonWrist;

public class Wrist extends CommonWrist {
    Servo pitch;
    private static final double PITCH_OFFSET = 3;

    public Wrist(Servo pitch){
        super(pitch);
        this.pitch = pitch;
    }
    public void manualControlPitch(double stepSize){
        double targetPosition = stepSize + pitch.getPosition();
        if (targetPosition > 1 - CommonWrist.SERVO_OFFSET){
            targetPosition = 1 - CommonWrist.SERVO_OFFSET;
        }
        if (targetPosition < CommonWrist.SERVO_OFFSET){
            targetPosition = CommonWrist.SERVO_OFFSET;
        }
        super.presetPositionPitch(targetPosition);
    }

}
