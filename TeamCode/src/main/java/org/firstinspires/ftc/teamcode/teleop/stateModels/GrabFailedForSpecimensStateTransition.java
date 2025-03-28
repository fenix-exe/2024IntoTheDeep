package org.firstinspires.ftc.teamcode.teleop.stateModels;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabFailedForSpecimensStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        BACK_TO_INTAKE_POSITION
    }

    private TransitionSteps dropSpecimenState;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    DriverControls driverControls;
    public GrabFailedForSpecimensStateTransition(Wrist wrist, IIntake intake, Arm arm, DriverControls driverControls){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driverControls = driverControls;
        dropSpecimenState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        dropSpecimenState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch(dropSpecimenState){
            case START:
                if (driverControls.enterIntakePosition() && FSMManager.robotState == RobotState.READY_TO_GO_TO_CLIP_POSITION) {
                    FSMManager.stopTransitions();
                    arm.holdArm();
                    intake.outtake();
                    wrist.presetPositionPitch(StateModelParameters.PickupSpecimensStateParameters.pitch);
                    arm.moveElbowToAngle(StateModelParameters.PickupSpecimensStateParameters.elbowAngle);
                    arm.moveSlideToLength(StateModelParameters.PickupSpecimensStateParameters.slideLength);
                    dropSpecimenState = TransitionSteps.BACK_TO_INTAKE_POSITION;
                }
                break;
            case BACK_TO_INTAKE_POSITION:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE
                        && Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    FSMManager.robotState = RobotState.READY_TO_GRAB_SPECIMEN;
                    dropSpecimenState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(dropSpecimenState == TransitionSteps.START);
    }
}
