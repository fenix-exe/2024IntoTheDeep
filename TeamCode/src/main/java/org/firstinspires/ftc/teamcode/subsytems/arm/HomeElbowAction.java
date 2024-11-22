package org.firstinspires.ftc.teamcode.subsytems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
                arm.setElbowPower(-0.5);
            }
            while (arm.elbow.limitSwitch.isPressed() && !cancelled){
                arm.setElbowPower(0.5);
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
