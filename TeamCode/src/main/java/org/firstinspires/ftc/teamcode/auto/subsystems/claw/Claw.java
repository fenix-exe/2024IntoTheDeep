package org.firstinspires.ftc.teamcode.auto.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ClawMessage;
import org.firstinspires.ftc.teamcode.common.CommonClaw;

import androidx.annotation.NonNull;

public class Claw extends CommonClaw {

    //This class is used to control only the intake(claw) in servo positions

    //initialize objects
    public Servo clawServo;
    private final DownsampledWriter clawWriter;

    public Claw(Servo claw) {
        super(claw);
        clawWriter = new DownsampledWriter("CLAW INFO", 50_000_000);
    }

    /* this action sets claw servo position using servo
    * finishes when claw is set to position
    * nothing happens when the action finishes
     */
    public class clawControl implements Action {
        private final double clawPos;

        clawControl(double clawPos){
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setClawPosition(clawPos);

            clawWriter.write(new ClawMessage(clawPos));

            return false;
        }
    }

    public Action clawControl(double clawPos){
        return new clawControl(clawPos);
    }
}
