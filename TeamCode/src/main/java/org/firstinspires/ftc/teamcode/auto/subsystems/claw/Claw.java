package org.firstinspires.ftc.teamcode.auto.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ClawMessage;
import org.firstinspires.ftc.teamcode.common.CommonClaw;

import androidx.annotation.NonNull;

public class Claw extends CommonClaw {
    public Servo clawServo;
    private final DownsampledWriter clawWriter;

    public Claw(Servo claw) {
        super(claw);
        clawWriter = new DownsampledWriter("CLAW INFO", 50_000_000);
    }

    public class clawControl implements Action {
        private final double clawPos;

        clawControl(double clawPos){
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawWriter.write(new ClawMessage(clawPos));
            setClawPosition(clawPos);
            return false;
        }
    }

    public Action clawControl(double clawPos){
        return new clawControl(clawPos);
    }
}
