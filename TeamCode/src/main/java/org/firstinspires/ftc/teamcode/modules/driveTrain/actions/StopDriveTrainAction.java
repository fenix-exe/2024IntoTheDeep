package org.firstinspires.ftc.teamcode.modules.driveTrain.actions;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class StopDriveTrainAction implements IRobotAction {
    boolean cancelled = false;
    boolean complete = false;
    DriveTrain driveTrain;
    public StopDriveTrainAction(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!isCancelled()){
            driveTrain.stopDriveTrain();
        }
        this.complete = true;
    }

    @Override
    public boolean isComplete() {
        return this.complete;
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
