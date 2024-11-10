package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import androidx.annotation.NonNull;

@Config
public class differential {
    ServoImplEx left;
    ServoImplEx right;

    float leftRange = 300;
    float rightRange = 300;
    public static float pitchError = -13;
    public static float rollError = -18;

    public differential(ServoImplEx left, ServoImplEx right){
        this.left = left;
        this.right = right;
    }

    private double setAPosition(double roll, double pitch) {
        double aPos;
        aPos = (1 / leftRange) * (pitch + roll / 2) + 0.5;
        return aPos;
    }

    private double setBPosition(double roll, double pitch) {
        double bPos;
        bPos = (1 / rightRange) * (pitch - roll / 2) + 0.5;
        return bPos;
    }

    public void setDifferentialPosition(double pitch, double roll){
        left.setPosition(setAPosition(-2*(pitch)+pitchError, roll+rollError));
        right.setPosition(setBPosition(-2*(pitch)+pitchError, roll+rollError));
    }

    public class setDiffy implements Action {
        private final double pitch;
        private final double roll;

        public setDiffy(double pitch, double roll) {
            this.pitch = pitch;
            this.roll = roll;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setDifferentialPosition(pitch, roll);
            return true;
        }
    }

    public Action setDiffy(double pitch, double roll) {
        return new setDiffy(pitch, roll);
    }

    

}
