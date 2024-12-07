package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveToPresetPositionAction implements IRobotAction {
    Arm arm;
    ArmPresetPosition presetPosition;
    private boolean manual_override_arm_rules;
    public boolean cancelled = false;

    public MoveToPresetPositionAction(Arm arm, ArmPresetPosition presetPosition, boolean manual_override_arm_rules){
        this.manual_override_arm_rules = manual_override_arm_rules;
        this.arm = arm;
        this.presetPosition = presetPosition;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if(!cancelled) {
            arm.moveToPresetPosition(presetPosition, manual_override_arm_rules);
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
