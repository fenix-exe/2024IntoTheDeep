package org.firstinspires.ftc.teamcode.teleop.stateModels;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToClipSpecimenStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        MOVING_SLIDES_AND_WRIST_TO_DEPOSIT
    }

    private TransitionSteps goToClipState;
    Wrist wrist;
    Arm arm;
    DriveControlMap driverControls;
    public GoToClipSpecimenStateTransition(Wrist wrist, Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        goToClipState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        goToClipState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch(goToClipState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && FSMManager.robotState == RobotState.READY_TO_GO_TO_CLIP_POSITION){
                    FSMManager.stopTransitions();
                    arm.moveSlideToLength(StateModelParameters.DepositSpecimenPositionStateParameters.slideLength);
                    goToClipState = TransitionSteps.MOVING_SLIDES_AND_WRIST_TO_DEPOSIT;
                }
                break;
            case MOVING_SLIDES_AND_WRIST_TO_DEPOSIT:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    FSMManager.robotState = RobotState.READY_TO_DEPOSIT_CLIP;
                    goToClipState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(goToClipState == TransitionSteps.START);
    }
}
