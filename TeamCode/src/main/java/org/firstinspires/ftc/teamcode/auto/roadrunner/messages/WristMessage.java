package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class WristMessage {
    public long timestamp;
    public double pitchPosition;
    public double rollPosition;

    public WristMessage(double pitchPosition, double rollPosition) {
        this.timestamp = System.nanoTime();
        this.pitchPosition = pitchPosition;
        this.rollPosition = rollPosition;
    }
}
