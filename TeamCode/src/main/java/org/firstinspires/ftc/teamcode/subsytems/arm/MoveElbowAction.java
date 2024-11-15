package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveElbowAction implements IRobotAction {
    Arm arm;
    double joystickMovement;
    boolean cancelled = false;
    boolean complete = false;

    public MoveElbowAction(Arm arm, double joystickMovement){
        this.arm = arm;
        this.joystickMovement = joystickMovement;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!this.cancelled){
            arm.moveElbow(joystickMovement);
        }
        this.complete = true;
    }

    @Override
    public boolean isComplete() {
        return this.complete;
    }

    @Override
    public boolean isCancelled() {
        return cancelled;
    }

    @Override
    public ActionType getActionType() {
        return ActionType.OTHER;
    }
}
