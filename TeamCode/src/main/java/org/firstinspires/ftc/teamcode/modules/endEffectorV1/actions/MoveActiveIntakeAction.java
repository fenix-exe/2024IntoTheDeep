package org.firstinspires.ftc.teamcode.modules.endEffectorV1.actions;


import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffector;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.ActiveIntakeDirection;

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
            } else if (direction == ActiveIntakeDirection.BACKWARD){
                endEffector.activeIntakeBackward();
            } else {
                endEffector.activeIntakeOff();
            }
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
