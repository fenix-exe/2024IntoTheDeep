package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToIntakeStateTransition implements IStateTransition {

    private enum TransitionSteps {
        START,
        WAITING_FOR_CLAW_TO_OPEN_TO_SAFELY_DEPOSIT,
        RETRACTING_SLIDES,
        MOVING_WRIST,
        MOVING_ELBOW,
        MOVING_SLIDE
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriveControlMap driverControls;
    public GoToIntakeStateTransition(Wrist wrist, IIntake intake, Arm arm, DriveControlMap driverControls){
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

        switch (intakeTransitionStep) {
            case START:
                boolean atStart = FSMManager.getInstance().robotState == RobotState.START;
                // Is the Robot at a deposit position holding onto a sample?
                boolean fromBucket = FSMManager.getInstance().robotState == RobotState.DRIVING_TO_SUBMERSIBLE;
                boolean readyToDepositToHumanPlayer = driverControls.specimenSampleIntake() && (FSMManager.getInstance().robotState == RobotState.DRIVING_TO_SUBMERSIBLE || FSMManager.getInstance().robotState == RobotState.START);
                if (((driverControls.depositBack() || driverControls.specimenSampleIntake())
                        &&(atStart || fromBucket || FSMManager.getInstance().robotState == RobotState.CLIPPED)) || (readyToDepositToHumanPlayer)) {
                    FSMManager.getInstance().stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    if (fromBucket){
                        wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                        arm.moveSlideToLength(StateModelParameters.IntakeStateParameters.slideLength);
                        intakeTransitionStep = TransitionSteps.MOVING_SLIDE;
                    } else if (readyToDepositToHumanPlayer){
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
            case RETRACTING_SLIDES:
                if(Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                    intakeTransitionStep = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 450) {
                    if (arm.getSlideExtension() < StateModelParameters.IntakeStateParameters.slideLength){
                        arm.moveSlideToLength(6);
                    }
                    timer.reset();
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(StateModelParameters.IntakeStateParameters.slideLength);
                    intakeTransitionStep = TransitionSteps.MOVING_SLIDE;
                }
                break;
            case MOVING_SLIDE:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    FSMManager.getInstance().robotState = RobotState.READY_TO_ENTER_SUBMERSIBLE;
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
