package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToGrabSpecimenPositionStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        OPENING_CLAW,
        MOVING_WRIST,
        MOVING_ELBOW_AND_SLIDE
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriverControls driverControls;
    public GoToGrabSpecimenPositionStateTransition(Wrist wrist, Claw claw, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        this.driverControls = driverControls;
        intakeTransitionStep = TransitionSteps.START;
    }
    @Override
    public void reset() {
        intakeTransitionStep = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (intakeTransitionStep) {
            case START:
                if (driverControls.pickupAndDepositSpecimens() &&
                        (FSMManager.robotState == RobotState.START
                                || FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE
                                || FSMManager.robotState == RobotState.INTERMEDIATE_DEPOSIT_TO_BUCKET_STATE
                                || FSMManager.robotState == RobotState.READY_TO_GO_TO_GRAB_SPECIMEN)) {
                    timer = new ElapsedTime();
                    FSMManager.stopTransitions();
                    timer.reset();
                    claw.openClaw();
                    intakeTransitionStep = TransitionSteps.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 200) {
                    timer.reset();
                    wrist.presetPosition(StateModelParameters.PickupSpecimensStateParameters.pitch, StateModelParameters.PickupSpecimensStateParameters.roll);
                    intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250) {
                    arm.moveSlideToLength(StateModelParameters.PickupSpecimensStateParameters.slideLength);
                    arm.moveElbowToAngle(StateModelParameters.PickupSpecimensStateParameters.elbowAngle);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW_AND_SLIDE;
                }
                break;
            case MOVING_ELBOW_AND_SLIDE:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE && Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    FSMManager.robotState = RobotState.READY_TO_GRAB_SPECIMEN;
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
