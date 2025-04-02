package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class EnterSubmersibleStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        PITCH_DOWN,
        WAIT_A_BIT
    }
    TransitionSteps steps;
    DriverControls controls;
    Wrist wrist;
    IIntake intake;
    ElapsedTime timer;
    public EnterSubmersibleStateTransition(Wrist wrist, IIntake intake, DriverControls controls){
        this.controls =controls;
        this.wrist=wrist;
        this.intake=intake;
        steps = TransitionSteps.START;
        timer=new ElapsedTime();
    }


    @Override
    public void reset() {
        steps = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch(steps){
            case START:
                if ((controls.depositBack() || controls.specimenSampleIntake()) && FSMManager.robotState == RobotState.READY_TO_ENTER_SUBMERSIBLE){
                    FSMManager.stopTransitions();
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.EnterSubmersibleStateParameters.pitch);
                    steps = TransitionSteps.PITCH_DOWN;
                }
                break;
            case PITCH_DOWN:
                if (timer.milliseconds() > 250){
                    timer.reset();
                    steps = TransitionSteps.WAIT_A_BIT;
                }
                break;
            case WAIT_A_BIT:
                if (timer.milliseconds() > StateModelParameters.EnterSubmersibleStateParameters.waitTime){
                    intake.intake();
                    FSMManager.robotState = RobotState.READY_TO_INTAKE_SAMPLE;
                    steps = TransitionSteps.START;
                }
                break;

        }
    }

    @Override
    public boolean inProgress() {
        return !(steps == TransitionSteps.START);
    }
}
