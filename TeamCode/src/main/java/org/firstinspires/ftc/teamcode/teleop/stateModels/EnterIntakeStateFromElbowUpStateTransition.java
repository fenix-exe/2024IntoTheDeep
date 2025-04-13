package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class EnterIntakeStateFromElbowUpStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        MOVE_ELBOW,
        EXTEND_SLIDES,
        MOVE_PITCH
    }
    private TransitionSteps goToIntakeState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    IIntake intake;
    DriveControlMap driverControls;
    public EnterIntakeStateFromElbowUpStateTransition(Wrist wrist, IIntake intake, Arm arm, DriveControlMap driverControls){
        this.intake = intake;
        this.wrist =wrist;
        this.arm =arm;
        this.driverControls = driverControls;
        goToIntakeState = TransitionSteps.START;
        timer = new ElapsedTime();
    }
    @Override
    public void reset() {
        goToIntakeState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (goToIntakeState){
            case START:
                if (driverControls.enterIntakePosition() && FSMManager.getInstance().robotState == RobotState.ELBOW_TO_DEPOSIT_IN_BUCKET){
                    FSMManager.getInstance().stopTransitions();
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    arm.moveSlideToLength(6);
                    goToIntakeState = TransitionSteps.MOVE_ELBOW;
                }
                break;
            case MOVE_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(StateModelParameters.IntakeStateParameters.slideLength);
                    goToIntakeState = TransitionSteps.EXTEND_SLIDES;
                }
                break;
            case EXTEND_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.EnterSubmersibleStateParameters.pitch);
                    goToIntakeState = TransitionSteps.MOVE_PITCH;
                }
                break;
            case MOVE_PITCH:
                if (timer.milliseconds() > 400){
                    intake.intake();
                    FSMManager.getInstance().robotState = RobotState.READY_TO_INTAKE_SAMPLE;
                    goToIntakeState = TransitionSteps.START;
                }

        }
    }

    @Override
    public boolean inProgress() {
        return false;
    }
}
