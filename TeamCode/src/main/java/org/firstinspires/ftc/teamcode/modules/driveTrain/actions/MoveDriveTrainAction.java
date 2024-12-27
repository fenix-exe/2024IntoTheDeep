package org.firstinspires.ftc.teamcode.modules.driveTrain.actions;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;

public class MoveDriveTrainAction implements IRobotAction {
    DriveTrain driveTrain;
    double speedMultiplier = -1;
    boolean complete = false;
    boolean cancelled = false;
    public MoveDriveTrainAction(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    public MoveDriveTrainAction(DriveTrain driveTrain, double speedMultiplier){
        this.driveTrain = driveTrain;
        this.speedMultiplier = speedMultiplier;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    public void execute(){
        if (!cancelled) {
            if (speedMultiplier == -1) {
                if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC) {
                    driveTrain.RobotCentric_Drive();
                } else {
                    driveTrain.FieldCentricDrive();
                }
            } else {
                if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC) {
                    driveTrain.RobotCentric_Drive(speedMultiplier);
                } else {
                    driveTrain.FieldCentricDrive(speedMultiplier);
                }
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

