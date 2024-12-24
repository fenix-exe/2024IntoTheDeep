package org.firstinspires.ftc.teamcode.modules.arm.actions;


import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class HomeElbowAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    Arm arm;
    public HomeElbowAction(Arm arm){
        this.arm=arm;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            while (!arm.elbow.limitSwitch.isPressed() && !cancelled){
                arm.setElbowPower(-0.65);
            }
            while (arm.elbow.limitSwitch.isPressed() && !cancelled){
                arm.setElbowPower(0.65);
            }
            arm.setElbowPower(0);

            arm.resetEncoders();
            completed = true;
        }
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
        return ActionType.ARM_MOVE_TO_PRESET;
    }
}