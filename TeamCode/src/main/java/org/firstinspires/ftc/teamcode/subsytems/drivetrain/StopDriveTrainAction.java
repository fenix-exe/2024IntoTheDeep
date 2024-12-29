package org.firstinspires.ftc.teamcode.subsytems.drivetrain;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

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
            if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
                driveTrain.RobotCentric_Drive();
            } else {
                driveTrain.FieldCentricDrive();
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
