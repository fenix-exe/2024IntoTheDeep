package org.firstinspires.ftc.teamcode.auto.subsystems.wrist;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.WristMessage;
import org.firstinspires.ftc.teamcode.common.CommonWrist;

import androidx.annotation.NonNull;

public class Wrist extends CommonWrist {

    //This class is used to control only the wrist(pitch and roll) in degrees

    Servo pitchLeft;
    Servo pitchRight;

    private final DownsampledWriter wristWriter;


    public Wrist(Servo pitch){
        super(pitch);
        this.pitchLeft = pitch;

        wristWriter = new DownsampledWriter("WRIST INFO", 50_000_000);
    }

    public Wrist(Servo pitchLeft, Servo pitchRight){
        super(pitchLeft, pitchRight);
        this.pitchLeft = pitchLeft;
        this.pitchRight = pitchRight;
        wristWriter = new DownsampledWriter("WRIST INFO", 50_000_000);
    }

    /* this action sets pitch and roll servos using degrees
     * finishes when pitch and roll servos is set to position
     * nothing happens when the action finishes
     */
    public class wristControl implements Action {
        private final double pitchPos;

        wristControl(double pitchPos){
            this.pitchPos = pitchPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            presetPositionPitch(pitchPos);
            wristWriter.write(new WristMessage(getPitchAngle(), 0));
            return false;
        }
    }

    public Action wristControl(double pitchPos){
        return new wristControl(pitchPos);
    }

}
