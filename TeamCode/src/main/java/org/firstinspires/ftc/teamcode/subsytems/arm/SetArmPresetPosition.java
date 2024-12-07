package org.firstinspires.ftc.teamcode.subsytems.arm;

import org.firstinspires.ftc.teamcode.robot.IRobotAction;

public class SetArmPresetPosition implements IRobotAction {
    boolean cancelled = false;
    boolean completed = false;
    ArmPresetPosition presetToChange;
    double elbowAngle;
    double slideLength;
    public SetArmPresetPosition(ArmPresetPosition presetToChange, double elbowAngle, double slideLength){
        this.presetToChange = presetToChange;
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }
    @Override
    public void cancel() {
        this.cancelled = true;
    }

    @Override
    public void execute() {
        if (!cancelled){

            switch(presetToChange.getName()){
                case INTAKE:
                    ArmPresetPosition.INTAKE_POSITION = new ArmPresetPosition(ArmPresetPositionNames.INTAKE,elbowAngle,slideLength);
                    break;
                case DEPOSIT_FRONT_TOP:
                    ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_FRONT_TOP,elbowAngle,slideLength);
                    break;
                case DEPOSIT_FRONT_BOTTOM:
                    ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_FRONT_BOTTOM,elbowAngle,slideLength);
                    break;
                case DEPOSIT_BACK_TOP:
                    ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_BACK_TOP,elbowAngle,slideLength);
                    break;
                case DEPOSIT_BACK_BOTTOM:
                    ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_BACK_BOTTOM,elbowAngle,slideLength);
                    break;
                case DRIVING:
                    ArmPresetPosition.SAFE_DRIVING_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DRIVING,elbowAngle,slideLength);
                    break;
                case ASCENT_2_HANG:
                    ArmPresetPosition.ASCENT_2_HANG = new ArmPresetPosition(ArmPresetPositionNames.ASCENT_2_HANG,elbowAngle,slideLength);
                    break;
                case FLAT_ELBOW:
                    ArmPresetPosition.FLAT_ELBOW = new ArmPresetPosition(ArmPresetPositionNames.FLAT_ELBOW,elbowAngle,slideLength);
                    break;
                case INTAKE_DOWN:
                    ArmPresetPosition.INTAKE_DOWN = new ArmPresetPosition(ArmPresetPositionNames.INTAKE_DOWN,elbowAngle,slideLength);
                    break;
            }

            presetToChange = new ArmPresetPosition(presetToChange.getName(), elbowAngle, slideLength);
            //ArmPresetPosition.SAFE_DRIVING_POSITION = new ArmPresetPosition(presetToChange.getName(), elbowAngle, slideLength);
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
