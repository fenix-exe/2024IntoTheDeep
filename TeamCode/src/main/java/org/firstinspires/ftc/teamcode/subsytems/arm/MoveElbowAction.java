package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveElbowAction implements IRobotAction {
    Arm arm;
    double joystickMovement;
    boolean cancelled = false;
    boolean complete = false;
    boolean remove_arm_rules;

    public MoveElbowAction(Arm arm, double joystickMovement, boolean remove_arm_rules){
        this.arm = arm;
        this.joystickMovement = joystickMovement;
        this.remove_arm_rules = remove_arm_rules;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!this.cancelled){
            arm.moveElbow(joystickMovement, remove_arm_rules);
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
