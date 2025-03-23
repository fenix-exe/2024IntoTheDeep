package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class ActiveIntakeMessage {
    public long timestamp;
    public double intakePower;

    public ActiveIntakeMessage(double intakePower) {
        this.timestamp = System.nanoTime();
        this.intakePower = intakePower;
    }
}
