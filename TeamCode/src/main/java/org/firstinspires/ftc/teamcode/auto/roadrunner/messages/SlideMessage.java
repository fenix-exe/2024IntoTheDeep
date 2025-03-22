package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class SlideMessage {
    public long timestamp;
    public double slidePosition;
    public double slideTargetPosition;

    public SlideMessage(double slidePosition, double slideTargetPosition) {
        this.timestamp = System.nanoTime();
        this.slidePosition = slidePosition;
        this.slideTargetPosition = slideTargetPosition;
    }
}
