package org.firstinspires.ftc.teamcode.modules.driveTrain.actions;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class ResetIMUAction implements IRobotAction {
    DriveTrain driveTrain;
    boolean complete = false;
    boolean cancelled = false;
    public ResetIMUAction(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if (!cancelled){
            driveTrain.resetIMU();
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
