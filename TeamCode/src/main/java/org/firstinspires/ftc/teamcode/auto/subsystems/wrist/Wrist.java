package org.firstinspires.ftc.teamcode.auto.subsystems.wrist;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.WristMessage;
import org.firstinspires.ftc.teamcode.commonCode.CommonWrist;

import androidx.annotation.NonNull;

public class Wrist extends CommonWrist {
    Servo pitch;
    Servo roll;
    private final DownsampledWriter wristWriter;

    public Wrist(Servo pitch, Servo roll){
        super(pitch, roll);
        wristWriter = new DownsampledWriter("WRIST INFO", 50_000_000);
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
            wristWriter.write(new WristMessage(getPitchAngle(), rollPos));
            presetPositionPitch(pitchPos);
            presetPositionRoll(rollPos);
            return false;
        }
    }

    public Action wristControl(double pitchPos, double rollPos){
        return new wristControl(pitchPos, rollPos);
    }

}
