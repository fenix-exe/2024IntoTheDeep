package org.firstinspires.ftc.teamcode.auto.subsytems.wrist;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

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
        double targetPosition = stepSizeInDegrees/180 + roll.getPosition();
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
        //divide by 300 to convert angles to servo positions for pitch
        //0.5 is the middle position of the servo, maximum of +150 to -150 degrees
        //divide by 180 to convert angles to servo positions for roll
        //0.5 is the middle position of the servo, maximum of -90 to 90 degrees
        presetPositionPitch(pitch/300 + 0.5);
        presetPositionRoll(roll/180 + 0.5);
    }

    public class wristControl implements Action {
        private final double pitchPos;
        private final double rollPos;

        wristControl(double pitchPos, double rollPos){
            this.pitchPos = pitchPos;
            this.rollPos = rollPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            presetPositionPitch(pitchPos);
            presetPositionRoll(rollPos);
            return false;
        }
    }

    public Action wristControl(double pitchPos, double rollPos){
        return new wristControl(pitchPos, rollPos);
    }

}
