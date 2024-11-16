package org.firstinspires.ftc.teamcode.robot;

public interface IRobotAction {

    public enum ActionType{
        OTHER,
        ARM_MOVE_TO_PRESET,
        ARM_MOVE_MANUAL,
        END_EFFECTOR_MOVE_TO_PRESET
    }

    public void cancel();
    public void execute();

    public boolean isComplete();

    public boolean isCancelled();

    public ActionType getActionType();

}
