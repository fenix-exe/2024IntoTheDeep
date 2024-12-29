package org.firstinspires.ftc.teamcode.modules.arm.actions;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class HoldSlideAction implements IRobotAction {
    Arm arm;
    boolean complete = false;
    boolean cancelled = false;
    public HoldSlideAction(Arm arm){
        this.arm=arm;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            arm.holdSlide();
        }
        this.complete = true;
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
