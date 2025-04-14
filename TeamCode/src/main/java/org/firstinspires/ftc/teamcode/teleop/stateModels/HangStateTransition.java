package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class HangStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        STEP_1,
        STEP_2,
        STEP_3,
        STEP_4_PT_1,
        STEP_4_PT_2

    }
    TransitionSteps hangState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    LinearActuator linearActuator;
    DriveControlMap driverControls;
    public HangStateTransition(Wrist wrist, Arm arm, DriveControlMap driverControls, LinearActuator linearActuator){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        this.linearActuator = linearActuator;
        hangState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        hangState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch(hangState){
            case START:
                if (driverControls.hang()
                        && !(FSMManager.getInstance().robotState == RobotState.READY_TO_INTAKE_SAMPLE)
                        && !(FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_CLIP)
                        && !(FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER)
                        && !(FSMManager.getInstance().robotState == RobotState.READY_TO_ENTER_SUBMERSIBLE)
                        && !(FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_IN_BUCKET)
                        && !(FSMManager.getInstance().robotState == RobotState.ELBOW_TO_DEPOSIT_IN_BUCKET)
                        && !(FSMManager.getInstance().robotState == RobotState.HANGING)){
                    FSMManager.getInstance().stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.Hang.pitch);
                    arm.moveSlideToLength(StateModelParameters.Hang.slideExtensionToMoveElbow);
                    arm.moveElbowToAngle(StateModelParameters.Hang.initialElbowAngle);
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.linearActuatorExtension);
                    FSMManager.getInstance().robotState = RobotState.HANGING;
                    hangState = TransitionSteps.STEP_1;
                }
                break;
            case STEP_1:
                if (driverControls.hang()
                        && (Math.abs(linearActuator.getLinearActuatorPositionInches() - linearActuator.getLinearActuatorTargetPositionInches()) < RobotConstants.LINEAR_ACTUATOR_TOLERANCE)
                        && (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)
                        && (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)){
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.intermediateLinearActuatorHeight);
                    arm.moveElbowToAngle(StateModelParameters.Hang.intermediateElbowAngle);
                    arm.moveSlideToLength(StateModelParameters.Hang.slideExtension);
                    hangState = TransitionSteps.STEP_2;
                }
                break;
            case STEP_2:
                if (driverControls.hang()
                        && (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)
                        && (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)){
                    arm.moveElbowToAngle(StateModelParameters.Hang.hangElbowAngle);
                    hangState = TransitionSteps.STEP_3;
                }
                break;
            case STEP_3:
                if (driverControls.hang()
                        && (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)){
                    arm.moveElbowToAngle(StateModelParameters.Hang.finalElbowAngle);
                    hangState = TransitionSteps.STEP_4_PT_1;
                }
                break;
            case STEP_4_PT_1:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(StateModelParameters.Hang.slideIntermediatePosition);
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.linearActuatorRetraction);
                    hangState = TransitionSteps.STEP_4_PT_1;
                }
                break;
            case STEP_4_PT_2:
                if ((Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)
                        && (Math.abs(linearActuator.getLinearActuatorPositionInches() - linearActuator.getLinearActuatorTargetPositionInches()) < RobotConstants.LINEAR_ACTUATOR_TOLERANCE)){
                    hangState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(hangState == TransitionSteps.START);
    }
}
