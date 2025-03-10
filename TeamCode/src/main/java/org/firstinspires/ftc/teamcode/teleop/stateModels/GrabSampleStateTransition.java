package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabSampleStateTransition implements IStateTransition {
    private enum TransitionSteps {
        START,
        ELBOW_DOWN,
        WAIT_ONE_SECOND,
        INTAKE_CLOSING,
        ELBOW_UP,
        SLIDES_BACK_TO_5_INCHES_FROM_MAX_EXTENSION,
        WRIST_MOVING_UP,
        SLIDES_RETRACTING,
        ELBOW_MOVING_UP
    }
    private TransitionSteps grabSampleState;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriveTrain driveTrain;
    DriverControls driverControls;
    public GrabSampleStateTransition(Wrist wrist, Claw claw, Arm arm, DriveTrain driveTrain, DriverControls driverControls){
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        grabSampleState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        grabSampleState = TransitionSteps.START;
        driveTrain.lockDriveTrain(false);
    }

    @Override
    public void execute() {
        switch (grabSampleState) {
            case START:
                if (driverControls.grabSampleFromOutside()
                        && FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE) {
                    FSMManager.stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    arm.moveElbowToAngle(StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeDownAngle);
                    wrist.presetPositionPitch(StateModelParameters.GrabBlockFromOutsideStateParameters.downPitch + arm.getElbowAngleInDegrees());
                    grabSampleState = TransitionSteps.ELBOW_DOWN;
                    driveTrain.stopDriveTrain();
                    driveTrain.lockDriveTrain(true);
                }
                break;
            case ELBOW_DOWN:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE) {
                    timer.reset();
                    grabSampleState = TransitionSteps.WAIT_ONE_SECOND;
                }
                break;
            case WAIT_ONE_SECOND:
                if (timer.milliseconds() > StateModelParameters.GrabBlockFromOutsideStateParameters.waitTime){
                    timer.reset();
                    claw.closeClaw();
                    grabSampleState = TransitionSteps.INTAKE_CLOSING;
                }
                break;
            case INTAKE_CLOSING:
                if (driverControls.letGoOfGrabSampleFromOutside()) {
                    arm.moveElbowToAngle(StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeUpAngle);
                    grabSampleState = TransitionSteps.ELBOW_UP;
                }
                break;
            case ELBOW_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE) {
                    if (arm.getSlideExtension() > arm.getMaximumSlideExtensionAllowedInInches() - 7) {  //allows us to not break the 42 inch extension limit
                        arm.moveSlideToLength(arm.getMaximumSlideExtensionAllowedInInches() - 8); //1 inch more back to guarantee that we do not break the 42 inch extension limit
                        grabSampleState = TransitionSteps.SLIDES_BACK_TO_5_INCHES_FROM_MAX_EXTENSION;
                    } else {
                        timer.reset();
                        wrist.presetPositionPitch(StateModelParameters.GrabBlockFromOutsideStateParameters.upPitch);
                        grabSampleState = TransitionSteps.WRIST_MOVING_UP;
                    }
                }
                break;
            case SLIDES_BACK_TO_5_INCHES_FROM_MAX_EXTENSION:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.GrabBlockFromOutsideStateParameters.upPitch);
                    grabSampleState = TransitionSteps.WRIST_MOVING_UP;
                }
                break;
            case WRIST_MOVING_UP:
                if (timer.milliseconds() > 250) {
                    driveTrain.lockDriveTrain(false);
                    FSMManager.robotState = RobotState.READY_TO_LEAVE_SUBMERSIBLE;
                    grabSampleState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(grabSampleState == TransitionSteps.START);
    }
}
