package org.firstinspires.ftc.teamcode.teleop.stateModels.autoPilot;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.stateModels.IStateTransition;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;

public class DriveToSubmersibleStateTransition implements IStateTransition {
    IDriveTrain driveTrain;

    private enum TransitionSteps{
        START,
        MOVING_ARM_AND_DRIVING_TO_SUBMERSIBLE,
        AT_SUBMERSIBLE
    }
    private TransitionSteps steps;
    private Pose2d submersiblePose;

    private Telemetry telemetry;

    public DriveToSubmersibleStateTransition(Pose2d submersiblePose, IDriveTrain driveTrain, Telemetry telemetry){
        this.submersiblePose = submersiblePose;
        this.driveTrain =driveTrain;
        steps=TransitionSteps.START;
        this.telemetry = telemetry;
    }
    @Override
    public void reset() {
        steps = TransitionSteps.START;
    }

    @Override
    public void execute() {
        telemetry.addData("DriveToSubmersibleStateTransition state", steps);
        switch(steps){
            case START:
                if (AutoPilotFSM.autoPilotState == AutoPilotState.START){
                    if (AutoPilotFSM.returnToSubmersible()) {
                        AutoPilotFSM.stopTransitions();
                        AutoPilotFSM.autoPilotState =  AutoPilotState.DRIVING_TO_SUBMERSIBLE;
                        driveTrain.Follow(submersiblePose);
                        steps = TransitionSteps.MOVING_ARM_AND_DRIVING_TO_SUBMERSIBLE;
                    }
                }
                break;
            case MOVING_ARM_AND_DRIVING_TO_SUBMERSIBLE:
                if (!driveTrain.isFollowingPath()){
                    steps = TransitionSteps.AT_SUBMERSIBLE;
                }
                break;
            case AT_SUBMERSIBLE:  //keeps the state machine here so that we don't go back to start at the end of the path
                break;

        }

    }

    @Override
    public boolean inProgress() {
        return !(steps == TransitionSteps.START);
    }
}
