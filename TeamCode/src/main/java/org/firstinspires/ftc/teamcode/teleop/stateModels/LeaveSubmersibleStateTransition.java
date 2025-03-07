package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class LeaveSubmersibleStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        RETRACTING_SLIDES
    }
    private TransitionSteps leaveSubmersibleState;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriveTrain driveTrain;
    DriverControls driverControls;
    public LeaveSubmersibleStateTransition(Wrist wrist, Claw claw, Arm arm, DriveTrain driveTrain, DriverControls driverControls){
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        leaveSubmersibleState = TransitionSteps.START;
    }
    @Override
    public void reset() {
        leaveSubmersibleState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (leaveSubmersibleState){
            case START:
                if (driverControls.depositBack() && FSMManager.robotState == RobotState.READY_TO_LEAVE_SUBMERSIBLE){
                    timer = new ElapsedTime();
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.DriveStateParameters.slideLength);
                    FSMManager.stopTransitions();
                    leaveSubmersibleState = TransitionSteps.RETRACTING_SLIDES;
                }
                break;
            case RETRACTING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    FSMManager.robotState = RobotState.INTERMEDIATE_DEPOSIT_TO_BUCKET_STATE;
                    leaveSubmersibleState = TransitionSteps.START;
                }
        }
    }

    @Override
    public boolean inProgress() {
        return !(leaveSubmersibleState == TransitionSteps.START);
    }
}
