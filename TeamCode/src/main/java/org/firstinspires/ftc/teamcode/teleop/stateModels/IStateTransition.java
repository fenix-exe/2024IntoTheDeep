package org.firstinspires.ftc.teamcode.teleop.stateModels;

public interface IStateTransition {

    void reset();
    void execute();

    //This transition is in progress
    boolean inProgress();

}

