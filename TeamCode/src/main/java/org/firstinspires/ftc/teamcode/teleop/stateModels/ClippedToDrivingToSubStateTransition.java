package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class ClippedToDrivingToSubStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        MOVING_TO_DRIVE_TO_SUB
    }

    private TransitionSteps clippedToDrivingToSubState;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriveControlMap driverControls;
    public ClippedToDrivingToSubStateTransition(Wrist wrist, IIntake intake, Arm arm, DriveControlMap driverControls){
        clippedToDrivingToSubState = TransitionSteps.START;
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driverControls = driverControls;
    }

    @Override
    public void reset() {

        clippedToDrivingToSubState = TransitionSteps.START;
    }
    @Override
    public void execute() {

        switch (clippedToDrivingToSubState) {
            case START:
                if (driverControls.depositBack() && FSMManager.getInstance().robotState == RobotState.CLIPPED){
                    arm.moveSlideToLength(StateModelParameters.DepositSampleIntoBucketStateParameters.slideLength);
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                    clippedToDrivingToSubState = TransitionSteps.MOVING_TO_DRIVE_TO_SUB;
                }
                break;
            case MOVING_TO_DRIVE_TO_SUB:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE
                        && Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    FSMManager.getInstance().robotState = RobotState.DRIVING_TO_SUBMERSIBLE;
                    clippedToDrivingToSubState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(clippedToDrivingToSubState == TransitionSteps.START);
    }
}
