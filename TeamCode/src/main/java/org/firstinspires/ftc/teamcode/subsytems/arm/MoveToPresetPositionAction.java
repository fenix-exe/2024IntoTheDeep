package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveToPresetPositionAction implements IRobotAction {
    Arm arm;
    ArmPresetPosition presetPosition;
    public boolean cancelled = false;

    public MoveToPresetPositionAction(Arm arm, ArmPresetPosition  presetPosition){
        this.arm = arm;
        this.presetPosition = presetPosition;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if(!cancelled) {
            arm.moveToPresetPosition(presetPosition);
        }
    }

    @Override
    public boolean isComplete() {
        return arm.isArmAtPresetPosition(presetPosition);
    }

    @Override
    public boolean isCancelled() {
        return cancelled;
    }

    @Override
    public ActionType getActionType() {
        return ActionType.ARM_MOVE_TO_PRESET;
    }

    public String toString(){
        return "MoveToPresetPositionAction("+presetPosition.elbowAngle+", "+presetPosition.slideLength+")";
    }
}
