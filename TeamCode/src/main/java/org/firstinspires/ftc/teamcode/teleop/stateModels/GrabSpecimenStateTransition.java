package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabSpecimenStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        MOVING_SLIDE_BACK,
        CLOSING_CLAW,
        MOVING_ELBOW
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    Claw claw;
    Arm arm;
    DriveTrain driveTrain;
    DriverControls driverControls;
    ColorSensor color;
    boolean closingClaw;
    public GrabSpecimenStateTransition(Wrist wrist, Claw claw, Arm arm, DriveTrain driveTrain, DriverControls driverControls, ColorSensor color){
        this.wrist = wrist;
        this.claw = claw;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        intakeTransitionStep = TransitionSteps.START;
        this.color = color;
        closingClaw = false;
    }
    @Override
    public void reset() {
        intakeTransitionStep = TransitionSteps.START;
        driveTrain.lockDriveTrain(false);
        closingClaw = false;
    }

    @Override
    public void execute() {
        switch(intakeTransitionStep){
            case START:
                timer = new ElapsedTime();
                double distance = color.getDistance(DistanceUnit.MM);
                if(FSMManager.robotState == RobotState.READY_TO_GRAB_SPECIMEN){
                    if (distance < 15){
                        closingClaw = false;
                        driveTrain.stopDriveTrain();
                        driveTrain.lockDriveTrain(true);
                        arm.moveSlide(-1,false);
                        intakeTransitionStep = TransitionSteps.MOVING_SLIDE_BACK;
                    }
                    if (driverControls.pickupAndDepositSpecimens()){
                        timer.reset();
                        claw.closeClaw();
                        intakeTransitionStep = TransitionSteps.CLOSING_CLAW;
                    }
                }
                break;
            case MOVING_SLIDE_BACK:
                double distanceToSample = color.getDistance(DistanceUnit.MM);
                if (distanceToSample > 35 && !closingClaw){
                    timer.reset();
                    claw.closeClaw();
                    closingClaw = true;
                }
                if (distanceToSample > 45 || arm.getSlideExtension() < RobotConstants.LOW_SLIDE_TOLERANCE){
                    if (!closingClaw){
                        timer.reset();
                        claw.closeClaw();
                    }
                    closingClaw = false;
                    arm.moveSlide(0,false);
                    intakeTransitionStep = TransitionSteps.CLOSING_CLAW;
                }
                break;
            case CLOSING_CLAW:
                if (timer.milliseconds() > 200){
                    timer.reset();
                    arm.moveElbowToAngle(StateModelParameters.PickupSpecimensStateParameters.elbowUpAngle);
                    intakeTransitionStep = TransitionSteps.MOVING_ELBOW;
                }
                if (driverControls.enterIntakePosition()){
                    driveTrain.lockDriveTrain(false);
                    arm.holdArm();
                    claw.openClaw();
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE){
                    driveTrain.lockDriveTrain(false);
                    FSMManager.robotState = RobotState.READY_TO_GO_TO_CLIP_POSITION;
                    intakeTransitionStep = TransitionSteps.START;
                }
                if (timer.milliseconds() > 100){
                    driveTrain.lockDriveTrain(false);
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(intakeTransitionStep == TransitionSteps.START);
    }
}
