package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

public class MoveEndEffectorThroughJoystick implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    EndEffectorMovement direction;
    EndEffector endEffector;
    double deltaPitch;
    double deltaRoll;
    double STEP_SIZE = 3;
    public MoveEndEffectorThroughJoystick(EndEffector endEffector, EndEffectorMovement direction){
        this.endEffector = endEffector;
        this.direction = direction;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            switch(direction){
                case UP:
                    deltaPitch = STEP_SIZE;
                    deltaRoll = 0;
                    break;
                case DOWN:
                    deltaPitch = -STEP_SIZE;
                    deltaRoll = 0;
                    break;
                case LEFT:
                    deltaPitch = 0;
                    deltaRoll = -STEP_SIZE;
                    break;
                case RIGHT:
                    deltaPitch = 0;
                    deltaRoll = STEP_SIZE;
                    break;
                default:
                    deltaRoll = 0;
                    deltaPitch = 0;
                    break;
            }
            endEffector.diffyJoystick(deltaPitch, deltaRoll);
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
        return ActionType.END_EFFECTOR_MOVE_MANUAL;
    }
}
