package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;

import kotlin.jvm.internal.MutablePropertyReference0;

public class MoveElbowUpToDepositionStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        MOVE_ELBOW_UP
    }
    private TransitionSteps moveElbowUpState;
    Wrist wrist;
    Arm arm;
    DriveControlMap driverControls;
    public MoveElbowUpToDepositionStateTransition(Wrist wrist, Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        moveElbowUpState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        moveElbowUpState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (moveElbowUpState){
            case START:
                if ((driverControls.depositBack() && FSMManager.getInstance().robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE)){
                    FSMManager.getInstance().stopTransitions();
                    wrist.presetPositionPitch(StateModelParameters.DepositStateParameters.intermediatePitch);
                    arm.moveElbowToAngle(StateModelParameters.DepositStateParameters.elbowAngle);
                    moveElbowUpState = TransitionSteps.MOVE_ELBOW_UP;
                }
                break;
            case MOVE_ELBOW_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    FSMManager.getInstance().robotState = RobotState.ELBOW_TO_DEPOSIT_IN_BUCKET;
                    moveElbowUpState = TransitionSteps.START;
                }
        }
    }

    @Override
    public boolean inProgress() {
        return !(FSMManager.getInstance().robotState == RobotState.START);
    }
}
