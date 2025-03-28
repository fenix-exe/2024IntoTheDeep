package org.firstinspires.ftc.teamcode.teleop.subsytems.intake;


public interface IIntake {
    public enum IntakeDirection {FORWARD,BACKWARD,OFF}
    public void intake();
    public void outtake();
    public void slowOuttake();
    public void stop();
    public IntakeDirection getIntakeDirection();
}
