package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
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

    TransitionSteps intakeTransitionStep;
    ElapsedTime timer;
    Wrist wrist;
    IIntake intake;
    Arm arm;
    IDriveTrain driveTrain;
    DriveControlMap driverControls;
    public ColorSensor color;
    boolean intakeOn;
    public GrabSpecimenStateTransition(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriveControlMap driverControls, ColorSensor color){
        this.wrist = wrist;
        this.intake = intake;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.driverControls = driverControls;
        intakeTransitionStep = TransitionSteps.START;
        this.color = color;
        intakeOn = false;
        timer = new ElapsedTime();
    }
    @Override
    public void reset() {
        intakeTransitionStep = TransitionSteps.START;
        intakeOn = false;
    }

    @Override
    public void execute() {
        switch(intakeTransitionStep){
            case START:
                if(FSMManager.getInstance().robotState == RobotState.READY_TO_GRAB_SPECIMEN){
                    if(color != null){
                        color.updateDistance();
                        double distance = color.getDistance();
                        if (distance < 100){
                            intake.intake();
                        }
                        if (distance < 32.5){
                            FSMManager.getInstance().stopTransitions();
                            StateModelParameters.PickupSpecimensStateParameters.elbowAngle = arm.getElbowTargetPositionInDegrees() + arm.elbow.OFFSET;
                            timer.reset();
                            intake.stop();
                            intakeOn = false;
                            intakeTransitionStep = TransitionSteps.STOPPING_INTAKE;
                        }
                    } else {
                        intake.intake();
                    }
                    if (driverControls.pickupAndDepositSpecimens()){
                        FSMManager.getInstance().stopTransitions();
                        StateModelParameters.PickupSpecimensStateParameters.elbowAngle = arm.getElbowTargetPositionInDegrees() + arm.elbow.OFFSET;
                        timer.reset();
                        intake.stop();
                        intakeOn = false;
                        intakeTransitionStep = TransitionSteps.STOPPING_INTAKE;
                    }
                }
                break;
            case STOPPING_INTAKE:
                if (timer.milliseconds() > 0){
                    timer.reset();
                    wrist.presetPositionPitch(StateModelParameters.DepositSpecimenPositionStateParameters.pitch);
                    intakeTransitionStep = TransitionSteps.MOVING_PITCH;
                }
                break;
            case MOVING_PITCH:
                if (timer.milliseconds()>400){
                    FSMManager.getInstance().robotState = RobotState.READY_TO_GO_TO_CLIP_POSITION;
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
