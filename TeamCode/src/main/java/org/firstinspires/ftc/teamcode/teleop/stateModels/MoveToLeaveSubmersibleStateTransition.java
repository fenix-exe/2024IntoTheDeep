package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.opmodes.TeleOpBlue;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;
import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

public class MoveToLeaveSubmersibleStateTransition implements IStateTransition {
    private enum TransitionSteps {
        START,
        SLIDES_SLIGHTLY_IN,
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
    DriveControlMap driverControls;
    public ColorSensor colorSensor;
    Alliance alliance;
    public MoveToLeaveSubmersibleStateTransition(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriveControlMap driverControls, ColorSensor colorSensor, Alliance alliance){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        this.grabSampleState = TransitionSteps.START;
        this.colorSensor = colorSensor;
        this.alliance = alliance;
        timer = new ElapsedTime();
    }
    @Override
    public void reset() {
        grabSampleState = TransitionSteps.START;
    }

    public void debug(Telemetry telemetry){
        telemetry.addLine("Grab State: "+grabSampleState);
    }

    @Override
    public void execute() {
        switch (grabSampleState) {
            case START:
                if (FSMManager.getInstance().robotState == RobotState.READY_TO_INTAKE_SAMPLE){
                    if (driverControls.grabSampleFromOutside() || driverControls.depositBack() || driverControls.specimenSampleIntake()) {
                        caseStartMovementForSuccessfulPickup();
                    } else if (colorSensor != null && !driverControls.turnOffAutoGrab()) {
                        colorSensor.updateHSVandDistance();
                        colorSensor.updateDetectColor();
                        if (colorSensor.detectingYellow()){
                            caseStartMovementForSuccessfulPickup();
                        } else if (colorSensor.detectingBlue()) {
                            if (alliance == Alliance.BLUE){
                                caseStartMovementForSuccessfulPickup();
                            } else {
                                caseStartMovementForUnsuccesfulPickup();
                            }
                        } else if (colorSensor.detectingRed()) {
                            if (alliance == Alliance.RED){
                                caseStartMovementForSuccessfulPickup();
                            } else {
                                caseStartMovementForUnsuccesfulPickup();
                            }
                        }
                    }

                    /*if (colorSensor != null && !driverControls.turnOffAutoGrab()){
                        colorSensor.updateHSVandDistance();
                        colorSensor.updateDetectColor();
                        if (colorSensor.detectingYellow()){
                            caseStartMovementForSuccessfulPickup();
                        } else if (colorSensor.detectingBlue()){
                            if (alliance == Alliance.BLUE){
                                caseStartMovementForSuccessfulPickup();
                            } else {
                                caseStartMovementForUnsuccesfulPickup();
                            }
                        } else if (colorSensor.detectingRed()){
                            if (alliance == Alliance.RED){
                                caseStartMovementForSuccessfulPickup();
                            } else {
                                caseStartMovementForUnsuccesfulPickup();
                            }
                        }
                    }*/

                }
                if (FSMManager.getInstance().robotState == RobotState.READY_TO_ENTER_SUBMERSIBLE && driverControls.hang()){
                    caseStartMovementForSuccessfulPickup();
                }
                if (FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER && driverControls.escapePresets()){
                    caseStartMovementForSuccessfulPickup();
                }
                break;
            case EJECTION:
                if (timer.milliseconds() > 400){
                    timer.reset();
                    intake.stop();
                    grabSampleState = TransitionSteps.START;
                }
                break;
            case SLIDES_SLIGHTLY_IN:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.LOW_SLIDE_TOLERANCE){
                    timer.reset();
                    StateModelParameters.EnterSubmersibleStateParameters.pitch = wrist.getPitchAngle();
                    wrist.presetPositionPitch(StateModelParameters.LeaveSubmersibleStateParameters.pitch);
                    grabSampleState = TransitionSteps.PITCH_UP;
                }
                break;
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
                    FSMManager.getInstance().robotState = RobotState.READY_TO_LEAVE_SUBMERSIBLE;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(grabSampleState == TransitionSteps.START);
    }
    private void caseStartMovementForSuccessfulPickup(){
        FSMManager.getInstance().stopTransitions();
        if (arm.getSlideExtension() > 16){
            arm.moveSlideToLength(StateModelParameters.LeaveSubmersibleStateParameters.slideRetractionForPickupLength);
        } else {
            arm.moveSlideToLength(arm.getSlideExtension());
        }
        grabSampleState = TransitionSteps.SLIDES_SLIGHTLY_IN;
    }
    private void caseStartMovementForUnsuccesfulPickup(){
        FSMManager.getInstance().stopTransitions();
        timer.reset();
        intake.outtake();
        grabSampleState = TransitionSteps.EJECTION;
    }
}
