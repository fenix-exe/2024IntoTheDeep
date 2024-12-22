package org.firstinspires.ftc.teamcode.modules.arm.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveToPresetPositionAction implements IRobotAction {
    Arm arm;
    ArmPresetPosition presetPosition;
    private boolean manual_override_arm_rules;
    public boolean cancelled = false;
    ElapsedTime timer;
    double execTime;

    public MoveToPresetPositionAction(Arm arm, ArmPresetPosition presetPosition, boolean manual_override_arm_rules){
        this.manual_override_arm_rules = manual_override_arm_rules;
        this.arm = arm;
        this.presetPosition = presetPosition;
        this.timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void cancel() {
        this.cancelled = true;
        execTime = timer.milliseconds();
        timer = null;
    }

    public void execute(){
        if(!cancelled) {
            arm.moveToPresetPosition(presetPosition, manual_override_arm_rules);
        }
    }

    @Override
    public boolean isComplete() {
        boolean complete = arm.isArmAtPresetPosition(presetPosition);
        if (complete) {
            execTime = timer.milliseconds();
            timer = null;
        }
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
        return "MoveToPresetPositionAction("+presetPosition.elbowAngle+", "+presetPosition.slideLength+ ", " + execTime + ")";
    }
}
