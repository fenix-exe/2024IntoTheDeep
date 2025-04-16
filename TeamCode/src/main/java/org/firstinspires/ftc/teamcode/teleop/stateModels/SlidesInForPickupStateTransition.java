package org.firstinspires.ftc.teamcode.teleop.stateModels;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;
import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

public class SlidesInForPickupStateTransition implements IStateTransition{
    private enum TransitionSteps{
        START,
        SLIDES_IN
    }
    TransitionSteps steps;
    ColorSensor colorSensor;
    Arm arm;
    DriveControlMap driverControls;
    Alliance alliance;
    IIntake intake;
    public SlidesInForPickupStateTransition(DriveControlMap driverControls, Arm arm, ColorSensor colorSensor){
        this.driverControls = driverControls;
        this.arm = arm;
        this.colorSensor = colorSensor;
        steps = TransitionSteps.START;
    }
    @Override
    public void reset() {
        steps = TransitionSteps.START;
    }

    @Override
    public void execute() {
        switch (steps){
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
                }
                if (FSMManager.getInstance().robotState == RobotState.READY_TO_ENTER_SUBMERSIBLE && driverControls.hang()){
                    caseStartMovementForSuccessfulPickup();
                }
                if (FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_TO_HUMAN_PLAYER && driverControls.escapePresets()){
                    caseStartMovementForSuccessfulPickup();
                }
                break;
            case SLIDES_IN:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.LOW_SLIDE_TOLERANCE){
                    FSMManager.getInstance().robotState = RobotState.BRINGING_SLIDES_BACK_IN;
                    steps = TransitionSteps.START;
                }
        }
    }

    @Override
    public boolean inProgress() {
        return !(steps == TransitionSteps.START);
    }
    private void caseStartMovementForSuccessfulPickup(){
        arm.moveSlideToLength(StateModelParameters.LeaveSubmersibleStateParameters.slideRetractionForPickupLength);
        steps = TransitionSteps.SLIDES_IN;
    }
    private void caseStartMovementForUnsuccesfulPickup(){
        intake.outtake();
    }
}
