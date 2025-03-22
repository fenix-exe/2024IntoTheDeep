package org.firstinspires.ftc.teamcode.auto.subsystems.wrist;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commonCode.CommonWrist;

import androidx.annotation.NonNull;

public class Wrist extends CommonWrist {
    Servo pitch;
    Servo roll;

    public Wrist(Servo pitch, Servo roll){
        super(pitch, roll);
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
