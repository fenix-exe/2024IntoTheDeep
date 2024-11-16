package org.firstinspires.ftc.teamcode.util;

public class autoTeleTransfer {
    public static double getxPos() {
        return xPos;
    }

    public static void setxPos(double xPos) {
        autoTeleTransfer.xPos = xPos;
    }

    public static double getyPos() {
        return yPos;
    }

    public static void setyPos(double yPos) {
        autoTeleTransfer.yPos = yPos;
    }

    public static double getHeadingAngle() {
        return headingAngle;
    }

    public static void setHeadingAngle(double headingAngle) {
        autoTeleTransfer.headingAngle = headingAngle;
    }

    public static double getElbowTicks() {
        return elbowTicks;
    }

    public static void setElbowTicks(double elbowTicks) {
        autoTeleTransfer.elbowTicks = elbowTicks;
    }

    public static double getSlideTicks() {
        return slideTicks;
    }

    public static void setSlideTicks(double slideTicks) {
        autoTeleTransfer.slideTicks = slideTicks;
    }

    public static double getDifPitch() {
        return difPitch;
    }

    public static void setDifPitch(double difPitch) {
        autoTeleTransfer.difPitch = difPitch;
    }

    public static double getDifRoll() {
        return difRoll;
    }

    public static void setDifRoll(double difRoll) {
        autoTeleTransfer.difRoll = difRoll;
    }

    private static double xPos;
    private static double yPos;
    private static double headingAngle;
    private static double elbowTicks;
    private static double slideTicks;
    private static double difPitch;
    private static double difRoll;
}
