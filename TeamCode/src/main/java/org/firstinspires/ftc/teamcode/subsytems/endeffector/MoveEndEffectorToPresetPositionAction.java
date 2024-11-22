package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.Arm;

public class MoveEndEffectorToPresetPositionAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    EndEffector endEffector;
    EndEffectorPresetPosition endEffectorPresetPosition;
    Arm arm;
    public MoveEndEffectorToPresetPositionAction(Arm arm, EndEffector endEffector, EndEffectorPresetPosition endEffectorPresetPosition){
        this.endEffector=endEffector;
        this.endEffectorPresetPosition = endEffectorPresetPosition;
        this.arm=arm;
    }

    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if(!cancelled){
            if (endEffectorPresetPosition == EndEffectorPresetPosition.INTAKE_POSITION && arm.getElbowAngleInDegrees() > 5){
                return;  //protects from breaking diff when going from ascent 1 bar into submersible
            }
            endEffector.setDifferentialPosition(endEffectorPresetPosition.pitch, endEffectorPresetPosition.roll);
        }
        this.completed = true;
    }

    @Override
    public boolean isComplete() {
        return this.completed;
    }

    @Override
    public boolean isCancelled() {
        return this.cancelled;
    }

    @Override
    public ActionType getActionType() {
        return ActionType.END_EFFECTOR_MOVE_TO_PRESET;
    }
}
