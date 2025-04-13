package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabFailedStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        OPENING_CLAW,
        SLIDES_OUT,
        MOVING_WRIST_DOWN
    }
    private TransitionSteps grabFailedState;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriveControlMap driverControls;
    public GrabFailedStateTransition(Wrist wrist, IIntake intake, Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driverControls = driverControls;
        grabFailedState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        grabFailedState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (grabFailedState){
            case START:
                if (driverControls.enterIntakePosition() && FSMManager.getInstance().robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE){
                    FSMManager.getInstance().stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    intake.outtake();
                    grabFailedState = TransitionSteps.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 200){
                    arm.moveSlideToLength(StateModelParameters.DropBlockAndMoveWristDown.slideLength);
                    grabFailedState = TransitionSteps.SLIDES_OUT;
                }
                break;
            case SLIDES_OUT:
                if (Math.abs(arm.getSlideExtension()-arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    intake.stop();
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.EnterSubmersibleStateParameters.pitch);
                    grabFailedState = TransitionSteps.MOVING_WRIST_DOWN;
                }
                break;
            case MOVING_WRIST_DOWN:
                if (timer.milliseconds() > 250){
                    intake.intake();
                    FSMManager.getInstance().robotState = RobotState.READY_TO_INTAKE_SAMPLE;
                    grabFailedState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(grabFailedState == TransitionSteps.START);
    }
}
