package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class StatusMessage {

    public long timestamp;
    public String status;

    public StatusMessage(String status) {
        this.timestamp = System.nanoTime();
        this.status = status;
    }
}
