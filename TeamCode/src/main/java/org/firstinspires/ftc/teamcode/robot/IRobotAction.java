package org.firstinspires.ftc.teamcode.robot;

public interface IRobotAction {

    public enum ActionType{
        OTHER,
        ARM_MOVE_TO_PRESET,
        ARM_MOVE_MANUAL,
    }

    public void cancel();
    public void execute();

    public boolean isComplete();

    public boolean isCancelled();

    public ActionType getActionType();

}
