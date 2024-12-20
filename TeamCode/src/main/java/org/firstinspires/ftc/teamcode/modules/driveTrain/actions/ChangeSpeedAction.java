package org.firstinspires.ftc.teamcode.modules.driveTrain.actions;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class ChangeSpeedAction implements IRobotAction {

    DriveTrain driveTrain;
    boolean complete = false;
    boolean cancelled = false;
    public ChangeSpeedAction(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }


    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute() {
        if (driveTrain.speedMultiplier == 1) {
            driveTrain.speedMultiplier = 0.5;
        } else {
            driveTrain.speedMultiplier = 1;
        }
        this.complete = true;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    public boolean isCancelled() {
        return cancelled;
    }

    @Override
    public ActionType getActionType() {
        return null;
    }
}
