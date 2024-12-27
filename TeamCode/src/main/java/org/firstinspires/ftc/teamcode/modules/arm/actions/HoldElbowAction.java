package org.firstinspires.ftc.teamcode.modules.arm.actions;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class HoldElbowAction implements IRobotAction {
    Arm arm;
    boolean cancelled = false;
    boolean complete = false;
    public HoldElbowAction(Arm arm){
        this.arm=arm;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            arm.holdElbow();
        }
        this.complete=true;
    }

    @Override
    public boolean isComplete() {
        return this.complete;
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
