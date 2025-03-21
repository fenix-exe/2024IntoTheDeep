package org.firstinspires.ftc.teamcode.teleop.stateModels.autoPilot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.stateModels.FSMManager;
import org.firstinspires.ftc.teamcode.teleop.stateModels.IStateTransition;
//import org.firstinspires.ftc.teamcode.teleop.stateModels.RobotState;
import org.firstinspires.ftc.teamcode.teleop.stateModels.RobotState;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;

public class DriveToClipFSM implements IStateTransition {
    IDriveTrain driveTrain;
    private enum TransitionSteps{
        START,
        START_CYCLE,
        DRIVING_TO_SUBMERSIBLE,
        CLIPPING,
        LET_GO_OF_CLIP,
        BACK_TO_HUMAN_PLAYER
    }
    TransitionSteps steps;
    Pose2d almostClippingPose;
    Pose2d clippingPose;
    Pose2d submersiblePose;
    ElapsedTime timer;

    Telemetry telemetry;

    public DriveToClipFSM(Pose2d almostClippingPose, Pose2d clippingPose, Pose2d submersiblePose, IDriveTrain driveTrain, Telemetry telemetry){
        this.almostClippingPose = almostClippingPose;
        this.clippingPose = clippingPose;
        this.submersiblePose = submersiblePose;
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        steps = TransitionSteps.START;
    }
    @Override
    public void reset() {
        steps = TransitionSteps.START;
    }

    @Override
    public void execute() {
        telemetry.addData("DriveToClipFSM state:", steps);
        switch (steps){
            case START:
                if (AutoPilotFSM.autoPilotState == AutoPilotState.START){
                    //if (FSMManager.readyToGoToClip()){
                        AutoPilotFSM.stopTransitions();
                        AutoPilotFSM.autoPilotState = AutoPilotState.CYCLING_CLIPS;
                        steps = TransitionSteps.START_CYCLE;
                    //}
                }
                break;
            case START_CYCLE:
                if (AutoPilotFSM.autoPilotState == AutoPilotState.CYCLING_CLIPS){
                    //if (FSMManager.readyToGoToClip()){
                        timer = new ElapsedTime();
                        driveTrain.Follow(almostClippingPose);
                        steps = TransitionSteps.DRIVING_TO_SUBMERSIBLE;
                    //}
                }
                break;
            case DRIVING_TO_SUBMERSIBLE:
                if (timer != null && timer.milliseconds() > 500){
                    AutoPilotFSM.setGoToClip(true);
                    timer = null;
                }
                if (!driveTrain.isFollowingPath()){
                    driveTrain.Follow(clippingPose);
                    steps = TransitionSteps.CLIPPING;
                }
                break;
            case CLIPPING:
                if (!driveTrain.isFollowingPath()){
                    AutoPilotFSM.setDepositClip(true);
                    steps = TransitionSteps.LET_GO_OF_CLIP;
                }
                break;
            case LET_GO_OF_CLIP:
                //if (FSMManager.readyToGoToPickupClip()){
                    AutoPilotFSM.setGoToPickupClip(true);
                    driveTrain.Follow(submersiblePose);
                    steps = TransitionSteps.BACK_TO_HUMAN_PLAYER;
                //}
                break;
            case BACK_TO_HUMAN_PLAYER:
                if (!driveTrain.isFollowingPath()){
                    steps = TransitionSteps.START_CYCLE;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return false;
    }
}
