package org.firstinspires.ftc.teamcode.subsytems.endeffector;


import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveActiveIntakeAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    EndEffector endEffector;
    ActiveIntakeDirection direction;
    public MoveActiveIntakeAction(EndEffector endEffector, ActiveIntakeDirection direction){
        this.endEffector = endEffector;
        this.direction = direction;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            if(direction == ActiveIntakeDirection.FORWARD){
                endEffector.activeIntakeForward();
            } else {
                endEffector.activeIntakeBackward();
            }
        }
        this.completed = true;
    }

    @Override
    public boolean isComplete() {
        return false;
    }

    @Override
    public boolean isCancelled() {
        return false;
    }

    @Override
    public ActionType getActionType() {
        return null;
    }
}
