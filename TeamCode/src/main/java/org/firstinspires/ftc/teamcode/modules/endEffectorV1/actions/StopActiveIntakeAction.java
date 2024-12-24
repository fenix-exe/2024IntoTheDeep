package org.firstinspires.ftc.teamcode.modules.endEffectorV1.actions;

import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffector;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class StopActiveIntakeAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    EndEffector endEffector;
    public StopActiveIntakeAction(EndEffector endEffector){
        this.endEffector = endEffector;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if(!cancelled){
            endEffector.activeIntakeOff();
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
        return null;
    }
}