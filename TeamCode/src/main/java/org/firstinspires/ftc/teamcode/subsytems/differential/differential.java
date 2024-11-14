package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import androidx.annotation.NonNull;

@Config
public class differential {
    ServoImplEx left;
    ServoImplEx right;

    float leftRange = 270;
    float rightRange = 270;
    public static float pitchError = -15;
    public static float rollError = 30;


    public differential(ServoImplEx left, ServoImplEx right){
        this.left = left;
        this.right = right;
        left.setPwmRange(new PwmControl.PwmRange(600,2400));
        right.setPwmRange(new PwmControl.PwmRange(600,2400));
    }

    private double setAPosition(double pitch, double roll) {
        double aPos;
        aPos = (1 / leftRange) * (roll + ( pitch / 2)) + 0.5;
        return aPos;
    }

    private double setBPosition(double pitch, double roll) {
        double bPos;
        bPos = (1 / rightRange) * (roll - (pitch / 2)) + 0.5;
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
            return false;
        }
    }

    public Action setDiffy(double pitch, double roll) {
        return new setDiffy(pitch, roll);
    }

    

}
