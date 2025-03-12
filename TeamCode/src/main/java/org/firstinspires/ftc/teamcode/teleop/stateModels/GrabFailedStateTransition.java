package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabFailedStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        OPENING_CLAW,
        MOVING_WRIST_DOWN
    }
    private TransitionSteps grabFailedState;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriverControls driverControls;
    public GrabFailedStateTransition(Wrist wrist, Claw claw, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.claw = claw;
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
                if (driverControls.enterIntakePosition() && FSMManager.robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE){
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    grabFailedState = TransitionSteps.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 200){
                    timer.reset();
                    wrist.presetPosition(StateModelParameters.DropBlockAndMoveWristDown.pitch, 0);
                    grabFailedState = TransitionSteps.MOVING_WRIST_DOWN;
                }
                break;
            case MOVING_WRIST_DOWN:
                if (timer.milliseconds() > 250){
                    FSMManager.robotState = RobotState.READY_TO_INTAKE_SAMPLE;
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
