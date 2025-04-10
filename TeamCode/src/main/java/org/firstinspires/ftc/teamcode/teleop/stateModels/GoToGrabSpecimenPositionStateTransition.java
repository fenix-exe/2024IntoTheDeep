package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToGrabSpecimenPositionStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        MOVING_WRIST,
        MOVING_ELBOW_AND_SLIDE
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriveControlMap driverControls;
    public GoToGrabSpecimenPositionStateTransition(Wrist wrist, IIntake intake, Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.intake = intake;
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
                if ((driverControls.pickupAndDepositSpecimens() &&
                        (FSMManager.robotState == RobotState.START
                                || FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE
                                || FSMManager.robotState == RobotState.INTERMEDIATE_DEPOSIT_TO_BUCKET_STATE
                                || FSMManager.robotState == RobotState.CLIPPED))
                        || FSMManager.robotState == RobotState.READY_TO_GO_TO_GRAB_SPECIMEN) {
                    timer = new ElapsedTime();
                    FSMManager.stopTransitions();
                    timer.reset();
                    intake.outtake();
                    wrist.presetPositionPitch(StateModelParameters.PickupSpecimensStateParameters.pitch);
                    intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250) {
                    intake.stop();
                    arm.moveSlideToLength(StateModelParameters.PickupSpecimensStateParameters.slideLength);
                    arm.moveElbowToAngle(StateModelParameters.PickupSpecimensStateParameters.elbowAngle);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW_AND_SLIDE;
                }
                break;
            case MOVING_ELBOW_AND_SLIDE:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE && Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    arm.setElbowPower(0);
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
