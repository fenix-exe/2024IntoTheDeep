package org.firstinspires.ftc.teamcode.subsytems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import androidx.annotation.NonNull;

public class autoClaw {
    ServoImplEx pitch;
    ServoImplEx roll;
    ServoImplEx claw;

    public enum CLAW_POS{
        ALIGNING,
        OPEN,
        CLOSE
    }


    public autoClaw(ServoImplEx pitch, ServoImplEx roll, ServoImplEx claw){
        this.pitch = pitch;
        this.roll = roll;
        this.claw = claw;

    }

    public void setPitch(double pos){
        pitch.setPosition(pos);
    }

    public void setRoll(double pos){
        roll.setPosition(pos);
    }


    public void setClaw(double pos){
        claw.setPosition(pos);
    }

    public class clawControl implements Action {
        private final double pitchPos;
        private final double rollPos;
        private final double clawPos;

        clawControl(double pitchPos, double rollPos, double clawPos){
            this.pitchPos = pitchPos;
            this.rollPos = rollPos;
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pitch.setPosition(pitchPos);
            roll.setPosition(rollPos);
            claw.setPosition(clawPos);
            return false;
        }
    }

    public Action clawControl(double pitchPos, double rollPos, double clawPos){
        return new clawControl(pitchPos, rollPos, clawPos);
    }



}
