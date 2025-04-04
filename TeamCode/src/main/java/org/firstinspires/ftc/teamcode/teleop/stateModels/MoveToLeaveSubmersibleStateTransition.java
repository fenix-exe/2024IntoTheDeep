package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;

public class MoveToLeaveSubmersibleStateTransition implements IStateTransition {
    private enum TransitionSteps {
        START,
        PITCH_UP,
        INTAKE_OFF,
        SLIDES_IN,
        EJECTION
    }
    private TransitionSteps grabSampleState;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    IDriveTrain driveTrain;
    DriverControls driverControls;
    ColorSensor colorSensor;
    Alliance alliance;
    public MoveToLeaveSubmersibleStateTransition(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriverControls driverControls, ColorSensor colorSensor, Alliance alliance){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        grabSampleState = TransitionSteps.START;
        this.colorSensor = colorSensor;
        this.alliance = alliance;
        timer = new ElapsedTime();
    }
    @Override
    public void reset() {
        grabSampleState = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (grabSampleState) {
            case START:
                if (FSMManager.robotState == RobotState.READY_TO_INTAKE_SAMPLE){
                    if (driverControls.grabSampleFromOutside()) {
                        FSMManager.stopTransitions();
                        timer.reset();
                        wrist.presetPositionPitch(StateModelParameters.LeaveSubmersibleStateParameters.pitch);
                        grabSampleState = TransitionSteps.PITCH_UP;
                    }
                    if (colorSensor != null){
                        colorSensor.updateHSVandDistance();
                        if (colorSensor.detectingYellow()){
                            FSMManager.stopTransitions();
                            timer.reset();
                            wrist.presetPositionPitch(StateModelParameters.LeaveSubmersibleStateParameters.pitch);
                            grabSampleState = TransitionSteps.PITCH_UP;
                        }
                        if (colorSensor.detectingBlue()){
                            if (alliance == Alliance.BLUE){
                                FSMManager.stopTransitions();
                                timer.reset();
                                wrist.presetPositionPitch(StateModelParameters.LeaveSubmersibleStateParameters.pitch);
                                grabSampleState = TransitionSteps.PITCH_UP;
                            } else {
                                timer.reset();
                                intake.outtake();
                                grabSampleState = TransitionSteps.EJECTION;
                            }
                        }
                        if (colorSensor.detectingRed()){
                            if (alliance == Alliance.RED){
                                FSMManager.stopTransitions();
                                timer.reset();
                                wrist.presetPositionPitch(StateModelParameters.LeaveSubmersibleStateParameters.pitch);
                                grabSampleState = TransitionSteps.PITCH_UP;
                            } else {
                                timer.reset();
                                intake.outtake();
                                grabSampleState = TransitionSteps.EJECTION;
                            }
                        }
                    }
                }
                break;
            case EJECTION:{
                if (timer.milliseconds() > 400){
                    timer.reset();
                    intake.stop();
                    grabSampleState = TransitionSteps.START;
                }
                break;
            }
            case PITCH_UP:
                if (timer.milliseconds() > 250) {
                    timer.reset();
                    intake.stop();
                    grabSampleState = TransitionSteps.INTAKE_OFF;
                }
                break;
            case INTAKE_OFF:
                if (timer.milliseconds() > 100){
                    timer.reset();
                    arm.moveSlideToLength(StateModelParameters.LeaveSubmersibleStateParameters.slideLength);
                    grabSampleState = TransitionSteps.SLIDES_IN;
                }
                break;
            case SLIDES_IN:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    grabSampleState = TransitionSteps.START;
                    FSMManager.robotState = RobotState.READY_TO_LEAVE_SUBMERSIBLE;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(grabSampleState == TransitionSteps.START);
    }
}
