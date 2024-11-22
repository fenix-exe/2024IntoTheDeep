package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveEndEffectorToPresetPositionAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    EndEffector endEffector;
    EndEffectorPresetPosition endEffectorPresetPosition;
    public MoveEndEffectorToPresetPositionAction(EndEffector endEffector, EndEffectorPresetPosition endEffectorPresetPosition){
        this.endEffector=endEffector;
        this.endEffectorPresetPosition = endEffectorPresetPosition;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if(!cancelled){
            endEffector.setDifferentialPosition(endEffectorPresetPosition.pitch, endEffectorPresetPosition.roll);
        }
        this.completed = true;
    }

    @Override
    public boolean isComplete() {
        return this.completed;
    }

    @Override
    public boolean isCancelled() {
        return this.cancelled;
    }

    @Override
    public ActionType getActionType() {
        return ActionType.END_EFFECTOR_MOVE_TO_PRESET;
    }
}
