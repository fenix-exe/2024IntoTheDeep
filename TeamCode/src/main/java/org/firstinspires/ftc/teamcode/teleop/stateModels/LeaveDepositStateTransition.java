package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class LeaveDepositStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        OUTTAKING,
        MOVING_WRIST,
        RETRACTING_SLIDES,
        MOVING_ELBOW
    }
    TransitionSteps steps;
    ElapsedTime timer;
    IIntake intake;
    Wrist wrist;
    Arm arm;
    DriverControls controls;
    public LeaveDepositStateTransition(Wrist wrist, IIntake intake, Arm arm,DriverControls controls){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.controls = controls;
        timer = new ElapsedTime();
        steps = TransitionSteps.START;
    }
    @Override
    public void reset() {

    }

    @Override
    public void execute() {
        switch(steps){
            case START:
                if (controls.depositBack() && FSMManager.robotState == RobotState.READY_TO_DEPOSIT_IN_BUCKET){
                    FSMManager.stopTransitions();
                    timer.reset();
                    intake.outtake();
                    steps = TransitionSteps.OUTTAKING;
                }
                break;
            case OUTTAKING:
                if (timer.milliseconds() > 400){
                    timer.reset();
                    intake.stop();
                    wrist.presetPositionPitch(StateModelParameters.IntakeStateParameters.pitch);
                    steps = TransitionSteps.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 400){
                    arm.moveSlideToLength(StateModelParameters.DepositSampleIntoBucketStateParameters.slideLength);
                    steps = TransitionSteps.RETRACTING_SLIDES;
                }
                break;
            case RETRACTING_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    steps = TransitionSteps.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    FSMManager.robotState = RobotState.DRIVING_TO_SUBMERSIBLE;
                    steps = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return false;
    }
}
