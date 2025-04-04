package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class GrabSpecimenStateTransition implements IStateTransition{
    private enum TransitionSteps {
        START,
        STOPPING_INTAKE,
        MOVING_PITCH
    }

    private TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    IDriveTrain driveTrain;
    DriverControls driverControls;
    ColorSensor color;
    boolean closingClaw;
    public GrabSpecimenStateTransition(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriverControls driverControls, ColorSensor color){
        this.wrist = wrist;
        this.intake = intake;
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
        closingClaw = false;
    }

    @Override
    public void execute() {
        switch(intakeTransitionStep){
            case START:
                if(FSMManager.robotState == RobotState.READY_TO_GRAB_SPECIMEN){
                    if(color != null){
                        color.updateHSVandDistance();
                        double distance = color.getDistance();
                        if (distance < 30){
                            FSMManager.stopTransitions();
                            timer = new ElapsedTime();
                            intake.stop();
                            driveTrain.stopDriveTrain();
                            intakeTransitionStep = TransitionSteps.STOPPING_INTAKE;
                        }
                    }
                    if (driverControls.pickupAndDepositSpecimens()){
                        FSMManager.stopTransitions();
                        timer = new ElapsedTime();
                        timer.reset();
                        intake.stop();
                        intakeTransitionStep = TransitionSteps.STOPPING_INTAKE;
                    }
                }
                break;
            case STOPPING_INTAKE:
                if (timer.milliseconds() > 50){
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.DepositSpecimenPositionStateParameters.pitch);
                    intakeTransitionStep = TransitionSteps.MOVING_PITCH;
                }
                if (driverControls.enterIntakePosition()){
                    arm.holdArm();
                    intake.outtake();
                }
                break;
            case MOVING_PITCH:
                if (timer.milliseconds()>400){
                    FSMManager.robotState = RobotState.READY_TO_GO_TO_CLIP_POSITION;
                    intakeTransitionStep = TransitionSteps.START;
                }
                break;
        }
    }

    @Override
    public boolean inProgress() {
        return !(intakeTransitionStep == TransitionSteps.START);
    }
}
