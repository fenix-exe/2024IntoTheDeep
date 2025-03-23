package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public class OldAutoClawMessage {
    public long timestamp;
    public double pitchPosition;
    public double rollPosition;
    public double clawPosition;

    public OldAutoClawMessage(double pitchPosition, double rollPosition, double clawPosition) {
        this.timestamp = System.nanoTime();
        this.pitchPosition = pitchPosition;
        this.rollPosition = rollPosition;
        this.clawPosition = clawPosition;
    }
}
