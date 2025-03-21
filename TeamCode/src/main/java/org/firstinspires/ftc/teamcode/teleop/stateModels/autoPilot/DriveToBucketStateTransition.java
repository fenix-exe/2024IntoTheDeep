package org.firstinspires.ftc.teamcode.teleop.stateModels.autoPilot;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.stateModels.FSMManager;
import org.firstinspires.ftc.teamcode.teleop.stateModels.IStateTransition;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;

public class DriveToBucketStateTransition implements IStateTransition {
    IDriveTrain driveTrain;
    Telemetry telemetry;
    private enum PathTransitionStep{
        START,
        BACKING_UP,
        MOVING_ARM_AND_DRIVING_TO_BUCKET,
        AT_BUCKET
    }
    private PathTransitionStep step;
    //private Pose2d backupPose;
    private Pose2d bucketPose;
    public DriveToBucketStateTransition( Pose2d bucketPose, IDriveTrain driveTrain, Telemetry telemetry){
        this.bucketPose = bucketPose;
        this.driveTrain = driveTrain;
        this.step = PathTransitionStep.START;
        this.telemetry = telemetry;
    }
    @Override
    public void reset() {
        this.step = PathTransitionStep.START;
    }

    @Override
    public void execute() {
        telemetry.addData("DriveToBucketStateTransition state:", step);
        switch(step){
            case START:
                if (AutoPilotFSM.autoPilotState == AutoPilotState.START){
                    //if (FSMManager.holdingSample()) {
                        AutoPilotFSM.stopTransitions();
                        AutoPilotFSM.autoPilotState =  AutoPilotState.DRIVING_TO_BUCKET;
                        if (!AutoPilotFSM.isSafeToGoToDeposit()){
                            Pose2d currentPose = driveTrain.getCurrentPose();
                            Pose2d backupPose = new Pose2d(currentPose.position.x-10,currentPose.position.y,Math.toRadians(-90));  //backup by 10 inches before we rotate
                            telemetry.addLine("Going to backup pose");
                            driveTrain.Follow(backupPose);
                            step = PathTransitionStep.BACKING_UP;
                            return;
                        } else {
                            telemetry.addLine("Going to bucket pose");
                            driveTrain.Follow(bucketPose);
                            step = PathTransitionStep.MOVING_ARM_AND_DRIVING_TO_BUCKET;
                            return;
                        }
                    //}
                }
                break;
            case BACKING_UP:
                if (!driveTrain.isFollowingPath()){
                    telemetry.addLine("Going to bucket pose");
                    driveTrain.Follow(bucketPose);
                    step = PathTransitionStep.MOVING_ARM_AND_DRIVING_TO_BUCKET;
                    return;
                }
                break;
            case MOVING_ARM_AND_DRIVING_TO_BUCKET:
                if (!driveTrain.isFollowingPath()){
                    telemetry.addLine("At Bucket Pose");
                    step = PathTransitionStep.AT_BUCKET;
                    return;
                }
                break;
            case AT_BUCKET:  //keeps the state machine here so that we don't go back to start at the end of the path
                break;
        }

    }

    @Override
    public boolean inProgress() {
        return !(step == PathTransitionStep.START);
    }
}
