package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class ClawMessage {
    public long timestamp;
    public double clawPosition;

    public ClawMessage(double clawPosition) {
        this.timestamp = System.nanoTime();
        this.clawPosition = clawPosition;
    }
}
