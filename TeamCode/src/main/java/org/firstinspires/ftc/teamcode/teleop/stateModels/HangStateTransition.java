package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class HangStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        LINEAR_ACTUATOR_UP,
        LINEAR_ACTUATOR_DOWN,
        ELBOW_TO_SLIDE_EXTENSION_POSITION,
        EXTENDING_SLIDES,
        ELBOW_TO_HANG_POSITION,
        SLIDES_RETRACT,
        ELBOW_TO_SAFE

    }
    TransitionSteps hangState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    LinearActuator linearActuator;
    DriverControls driverControls;
    public HangStateTransition(Wrist wrist, Arm arm, DriverControls driverControls, LinearActuator linearActuator){
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
                if (driverControls.hang()){
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.Hang.pitch);
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.linearActuatorExtension);
                    hangState = TransitionSteps.LINEAR_ACTUATOR_UP;
                }
                break;
            case LINEAR_ACTUATOR_UP:
                if ((Math.abs(linearActuator.getLinearActuatorPositionInches() - linearActuator.getLinearActuatorTargetPositionInches()) < RobotConstants.LINEAR_ACTUATOR_TOLERANCE)
                        && driverControls.hang()){
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.linearActuatorRetraction);
                    hangState = TransitionSteps.LINEAR_ACTUATOR_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = TransitionSteps.START;
                }
                break;
            case LINEAR_ACTUATOR_DOWN:
                if ((Math.abs(linearActuator.getLinearActuatorPositionInches() - linearActuator.getLinearActuatorTargetPositionInches()) < RobotConstants.LINEAR_ACTUATOR_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(StateModelParameters.Hang.initialElbowAngle);
                    hangState = TransitionSteps.ELBOW_TO_SLIDE_EXTENSION_POSITION;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = TransitionSteps.START;
                }
                break;
            case ELBOW_TO_SLIDE_EXTENSION_POSITION:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveSlideToLength(StateModelParameters.Hang.slideExtension);
                    hangState = TransitionSteps.EXTENDING_SLIDES;
                }

                break;
            case EXTENDING_SLIDES:
                if ((Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(StateModelParameters.Hang.hangElbowAngle);
                    hangState = TransitionSteps.ELBOW_TO_HANG_POSITION;
                }
                break;
            case ELBOW_TO_HANG_POSITION:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveSlideToLength(StateModelParameters.Hang.slideIntermediatePosition);
                    hangState = TransitionSteps.SLIDES_RETRACT;
                }
                break;
            case SLIDES_RETRACT:
                if (((Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE))
                        && (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(StateModelParameters.Hang.endElbowAngle);
                    linearActuator.goToTargetPositionInches(StateModelParameters.Hang.linearActuatorExtension);
                    hangState = TransitionSteps.ELBOW_TO_SAFE;
                }
                break;
            case ELBOW_TO_SAFE:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    FSMManager.robotState = RobotState.HANGING;
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
