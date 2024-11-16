package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class MoveSlideAction implements IRobotAction {
    Arm arm;
    double slideMovement;

    public boolean cancelled = false;

    public boolean completed = false;


    public MoveSlideAction(Arm arm, double slideMovement){
        this.arm=arm;
        this.slideMovement = slideMovement;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if(!this.cancelled){
            arm.moveSlide(slideMovement);
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
