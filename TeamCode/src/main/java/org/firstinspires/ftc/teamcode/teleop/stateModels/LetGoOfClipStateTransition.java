package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class LetGoOfClipStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        OPENING_CLAW,
        RETRACTING_SLIDES,
    }

    private TransitionSteps clipState;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriverControls driverControls;
    public LetGoOfClipStateTransition(Wrist wrist, IIntake intake, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.intake = intake;
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
                    intake.outtake();
                    clipState = TransitionSteps.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 300){
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.PickupSpecimensStateParameters.slideLength);
                    clipState = TransitionSteps.RETRACTING_SLIDES;
                }
                break;
            case RETRACTING_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    clipState = TransitionSteps.START;
                    FSMManager.robotState = RobotState.CLIPPED;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(clipState == TransitionSteps.START);
    }
}
