package org.firstinspires.ftc.teamcode.modules.arm.actions;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveSlideAction implements IRobotAction {
    Arm arm;
    double slideMovement;

    public boolean cancelled = false;

    public boolean completed = false;
    boolean remove_arm_rules;


    public MoveSlideAction(Arm arm, double slideMovement, boolean remove_arm_rules){
        this.arm=arm;
        this.slideMovement = slideMovement;
        this.remove_arm_rules = remove_arm_rules;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if(!this.cancelled){
            arm.moveSlide(slideMovement, remove_arm_rules);
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
        return ActionType.OTHER;
    }
}
