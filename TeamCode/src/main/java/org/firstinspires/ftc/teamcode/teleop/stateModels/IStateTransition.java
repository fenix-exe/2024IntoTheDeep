package org.firstinspires.ftc.teamcode.teleop.stateModels;

public interface IStateTransition {

    public void reset();
    public void execute();

    //This transition is in progress
    public boolean inProgress();

}

