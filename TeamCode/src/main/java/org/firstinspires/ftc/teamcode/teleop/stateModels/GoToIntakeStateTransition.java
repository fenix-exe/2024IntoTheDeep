package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToIntakeStateTransition implements IStateTransition {

    private enum TransitionSteps {
        START,
        WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT,
        MOVING_WRIST,
        MOVING_SLIDE,
        MOVING_ELBOW_AND_SLIDE
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriverControls driverControls;
    public GoToIntakeStateTransition(Wrist wrist, IIntake intake, Arm arm, DriverControls driverControls){
        intakeTransitionStep = TransitionSteps.START;
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driverControls = driverControls;
    }

    @Override
    public void reset() {
        intakeTransitionStep = TransitionSteps.START;
    }
    @Override
    public void execute() {
        boolean atStart = FSMManager.robotState == RobotState.START;
        // Is the Robot at a deposit position holding onto a sample?
        boolean readyToDepositInBucket = FSMManager.robotState == RobotState.READY_TO_DEPOSIT_IN_BUCKET;
        boolean readyToDepositToHumanPlayer = !driverControls.specimenSampleIntake() && FSMManager.robotState == RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER;
        switch (intakeTransitionStep) {
            case START:
                if (((driverControls.depositBack() || driverControls.specimenSampleIntake())
                        &&(atStart || readyToDepositInBucket || FSMManager.robotState == RobotState.READY_TO_GO_TO_GRAB_SPECIMEN)) || (readyToDepositToHumanPlayer)) {
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    if (readyToDepositInBucket || readyToDepositToHumanPlayer){
                        intake.outtake();
                        intakeTransitionStep = TransitionSteps.WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT;
                    } else {
                        wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                        intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                    }
                    //driveTrain.lockDriveTrain(false);
                }
                break;
            case WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT:
                if (timer.milliseconds() > 400){
                    intake.stop();
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                    intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 450) {
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.IntakeStateParameters.slideLength);
                    intakeTransitionStep = TransitionSteps.MOVING_SLIDE;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() < 12.5){
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW_AND_SLIDE;
                }
            case MOVING_ELBOW_AND_SLIDE:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE
                        && Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE) {
                    FSMManager.robotState = RobotState.READY_TO_ENTER_SUBMERSIBLE;
                    intakeTransitionStep = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(intakeTransitionStep == TransitionSteps.START);
    }

}
