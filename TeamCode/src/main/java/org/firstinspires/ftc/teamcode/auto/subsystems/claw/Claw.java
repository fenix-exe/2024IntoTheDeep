package org.firstinspires.ftc.teamcode.auto.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commonCode.CommonClaw;

import androidx.annotation.NonNull;

public class Claw extends CommonClaw {
    public Claw(Servo claw) {
        super(claw);
    }

    public class clawControl implements Action {
        private final double clawPos;

        clawControl(double clawPos){
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setClawPosition(clawPos);
            return false;
        }
    }

    public Action clawControl(double clawPos){
        return new clawControl(clawPos);
    }
}
