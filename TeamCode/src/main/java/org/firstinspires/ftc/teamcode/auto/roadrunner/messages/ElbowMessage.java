package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class ElbowMessage {
    public long timestamp;
    public double elbowPosition;
    public double elbowTargetPosition;
    public double elbowVoltage;

    public ElbowMessage(double elbowPosition, double elbowTargetPosition) {
        this.timestamp = System.nanoTime();
        this.elbowPosition = elbowPosition;
        this.elbowTargetPosition = elbowTargetPosition;
    }

    public ElbowMessage(double elbowPosition, double elbowTargetPosition, double elbowVoltage) {
        this.timestamp = System.nanoTime();
        this.elbowPosition = elbowPosition;
        this.elbowTargetPosition = elbowTargetPosition;
        this.elbowVoltage = elbowVoltage;
    }
}
