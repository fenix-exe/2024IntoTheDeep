package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class DepositSampleIntoObservationZoneStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        MOVE_ELBOW,
        EXTEND_SLIDES_AND_FIX_ROLL
    }
    private TransitionSteps depositSampleIntoObservationZoneState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    DriveControlMap driverControls;
    public DepositSampleIntoObservationZoneStateTransition(Wrist wrist, Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        depositSampleIntoObservationZoneState = TransitionSteps.START;
    }
    @Override
    public void reset() {

        depositSampleIntoObservationZoneState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (depositSampleIntoObservationZoneState){
            case START:
                if ((driverControls.specimenSampleIntake() && (FSMManager.getInstance().robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE || FSMManager.getInstance().robotState == RobotState.ELBOW_TO_DEPOSIT_IN_BUCKET))){
                    FSMManager.getInstance().stopTransitions();
                    timer = new ElapsedTime();
                    timer.reset();
                    arm.moveElbowToAngle(StateModelParameters.IntakeStateParameters.elbowAngle);
                    depositSampleIntoObservationZoneState = TransitionSteps.MOVE_ELBOW;
                }
                break;
            case MOVE_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.DepositSampleIntoObservationZone.extensionLength);
                    wrist.presetPositionPitch(StateModelParameters.DepositSampleIntoObservationZone.downPitch);
                    depositSampleIntoObservationZoneState = TransitionSteps.EXTEND_SLIDES_AND_FIX_ROLL;
                }
                break;
            case EXTEND_SLIDES_AND_FIX_ROLL:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    FSMManager.getInstance().robotState = RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER;
                    depositSampleIntoObservationZoneState = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(depositSampleIntoObservationZoneState == TransitionSteps.START);
    }
}
