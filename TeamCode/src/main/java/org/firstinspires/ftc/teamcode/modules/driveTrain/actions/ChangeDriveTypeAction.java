package org.firstinspires.ftc.teamcode.modules.driveTrain.actions;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class ChangeDriveTypeAction implements IRobotAction {
    DriveTrain driveTrain;
    boolean complete = false;
    boolean cancelled = false;
    public ChangeDriveTypeAction(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if (!cancelled){
            if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
                driveTrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
            } else{
                driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            }
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
