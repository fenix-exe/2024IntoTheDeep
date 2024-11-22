package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class SetArmPresetPosition implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    ArmPresetPosition presetToChange;
    double elbowAngle;
    double slideLength;
    public SetArmPresetPosition(ArmPresetPosition presetToChange, double elbowAngle, double slideLength){
        this.presetToChange = presetToChange;
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            presetToChange = new ArmPresetPosition(elbowAngle, slideLength);
        }
        completed = true;
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
        return null;
    }
}
