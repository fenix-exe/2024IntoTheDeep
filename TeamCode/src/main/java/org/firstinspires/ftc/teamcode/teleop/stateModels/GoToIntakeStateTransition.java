package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToIntakeStateTransition implements IStateTransition {

    private enum TransitionSteps {
        START,
        WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT,
        MOVING_WRIST,
        MOVING_ELBOW_AND_SLIDE
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriverControls driverControls;
    public GoToIntakeStateTransition(Wrist wrist, Claw claw, Arm arm, DriverControls driverControls){
        intakeTransitionStep = TransitionSteps.START;
        this.wrist = wrist;
        this.claw = claw;
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
                    claw.openClaw();
                    if (readyToDepositInBucket || readyToDepositToHumanPlayer){
                        intakeTransitionStep = TransitionSteps.WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT;
                    } else {
                        wrist.presetPosition(StateModelParameters.IntakeStateParameters.pitch, StateModelParameters.IntakeStateParameters.roll);
                        intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                    }
                    //driveTrain.lockDriveTrain(false);
                }
                break;
            case WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT:
                if (timer.milliseconds() > 250){
                    timer.reset();
                    wrist.presetPosition(StateModelParameters.IntakeStateParameters.pitch, StateModelParameters.IntakeStateParameters.roll);
                    intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250) {
                    timer.reset();
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    arm.moveSlideToLength(StateModelParameters.IntakeStateParameters.slideLength);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW_AND_SLIDE;
                }
                break;
            case MOVING_ELBOW_AND_SLIDE:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE
                        && Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE) {
                    FSMManager.robotState = RobotState.READY_TO_INTAKE_SAMPLE;
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
