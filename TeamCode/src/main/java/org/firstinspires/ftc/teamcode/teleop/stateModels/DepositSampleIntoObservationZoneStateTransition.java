package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class DepositSampleIntoObservationZoneStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        MOVE_ELBOW,
        EXTEND_SLIDES_AND_FIX_ROLL,
        WAIT_FOR_USER,
        DEPOSIT_SAMPLE,
        MOVE_TO_DRIVE_TO_SUB
    }
    private TransitionSteps depositSampleIntoObservationZoneState;
    ElapsedTime timer;
    Wrist wrist;
    Arm arm;
    IIntake intake;
    DriveControlMap driverControls;
    public DepositSampleIntoObservationZoneStateTransition(Wrist wrist, IIntake intake,Arm arm, DriveControlMap driverControls){
        this.wrist = wrist;
        this.arm = arm;
        this.driverControls = driverControls;
        this.intake = intake;
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
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE || !driverControls.specimenSampleIntake()){
                    arm.moveSlideToLength(arm.getSlideExtension());
                    FSMManager.getInstance().robotState = RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER;
                    depositSampleIntoObservationZoneState = TransitionSteps.WAIT_FOR_USER;
                }
                break;
            case WAIT_FOR_USER:
                if (!driverControls.specimenSampleIntake()){
                    timer.reset();
                    intake.outtake();
                    depositSampleIntoObservationZoneState = TransitionSteps.DEPOSIT_SAMPLE;
                }
                break;
            case DEPOSIT_SAMPLE:
                if (timer.milliseconds() > 400){
                    intake.stop();
                    arm.moveSlideToLength(StateModelParameters.DepositSampleIntoBucketStateParameters.slideLength);
                    depositSampleIntoObservationZoneState = TransitionSteps.MOVE_TO_DRIVE_TO_SUB;
                }
                break;
            case MOVE_TO_DRIVE_TO_SUB:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    FSMManager.getInstance().robotState = RobotState.DRIVING_TO_SUBMERSIBLE;
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
