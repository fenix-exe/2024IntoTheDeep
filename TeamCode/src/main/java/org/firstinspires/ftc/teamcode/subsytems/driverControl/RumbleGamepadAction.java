package org.firstinspires.ftc.teamcode.subsytems.driverControl;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;
import org.firstinspires.ftc.teamcode.subsytems.DriverControls;

public class RumbleGamepadAction implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    DriverControls controls;
    public RumbleGamepadAction(DriverControls controls){
        this.controls = controls;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){
            controls.rumbleArmGamepad();
        }
        completed = true;
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
        return null;
    }
}
