package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class LetGoOfClipStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        OPENING_CLAW,
        RETRACTING_SLIDES,
        MOVING_WRIST,
        MOVING_ELBOW
    }

    private TransitionSteps clipState;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriverControls driverControls;
    public LetGoOfClipStateTransition(Wrist wrist, Claw claw, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        this.driverControls = driverControls;
        clipState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        clipState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (clipState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && FSMManager.robotState == RobotState.READY_TO_DEPOSIT_CLIP){
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    clipState = TransitionSteps.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 250){
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.DepositSpecimensStateParameters.slideLength);
                    clipState = TransitionSteps.RETRACTING_SLIDES;
                }
                break;
            case RETRACTING_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    wrist.presetPosition(StateModelParameters.DepositSpecimensStateParameters.pitch, StateModelParameters.DepositSpecimensStateParameters.roll);
                    clipState = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveElbowToAngle(StateModelParameters.DepositSpecimensStateParameters.elbowAngle);
                    clipState = TransitionSteps.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    FSMManager.robotState = RobotState.READY_TO_GO_TO_GRAB_SPECIMEN;
                    clipState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(clipState == TransitionSteps.START);
    }
}
