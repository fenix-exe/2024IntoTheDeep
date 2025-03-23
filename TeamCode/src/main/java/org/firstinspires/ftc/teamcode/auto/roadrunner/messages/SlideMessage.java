package org.firstinspires.ftc.teamcode.auto.roadrunner.messages;

public final class SlideMessage {
    public long timestamp;
    public double slidePosition;
    public double slideTargetPosition;
    public double leftSlideVoltage;
    public double rightSlideVoltage;

    public SlideMessage(double slidePosition, double slideTargetPosition) {
        this.timestamp = System.nanoTime();
        this.slidePosition = slidePosition;
        this.slideTargetPosition = slideTargetPosition;
    }

    public SlideMessage(double slidePosition, double slideTargetPosition, double leftSlideVoltage, double rightSlideVoltage) {
        this.timestamp = System.nanoTime();
        this.slidePosition = slidePosition;
        this.slideTargetPosition = slideTargetPosition;
        this.leftSlideVoltage = leftSlideVoltage;
        this.rightSlideVoltage = rightSlideVoltage;
    }
}
