package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GoToDepositStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        MOVING_ELBOW_AND_WRIST,
        EXTENDING_SLIDES,
        MOVING_WRIST_TO_END
    }
    private TransitionSteps goToDepositState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    DriverControls driverControls;
    public GoToDepositStateTransition(Wrist wrist, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        goToDepositState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        goToDepositState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (goToDepositState){
            case START:
                if (driverControls.depositBack() && FSMManager.robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE){
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPositionPitch(0.5); //goes to 0.5 pitch to not catch on the bucket
                    arm.moveElbowToAngle(StateModelParameters.DepositStateParameters.elbowAngle);
                    goToDepositState = TransitionSteps.MOVING_ELBOW_AND_WRIST;
                }
                break;
            case MOVING_ELBOW_AND_WRIST:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveSlideToLength(StateModelParameters.DepositStateParameters.slideLength);
                    goToDepositState = TransitionSteps.EXTENDING_SLIDES;
                }
                break;
            case EXTENDING_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.DepositStateParameters.pitch);
                    goToDepositState = TransitionSteps.MOVING_WRIST_TO_END;
                }
                break;
            case MOVING_WRIST_TO_END:
                if (timer.milliseconds() > 250){
                    FSMManager.robotState = RobotState.READY_TO_DEPOSIT_IN_BUCKET;
                    goToDepositState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(goToDepositState == TransitionSteps.START);
    }
}
