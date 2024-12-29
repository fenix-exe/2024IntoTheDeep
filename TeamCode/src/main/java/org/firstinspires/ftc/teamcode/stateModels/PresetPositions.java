package org.firstinspires.ftc.teamcode.stateModels;

import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffectorPresetPositionNames;

public class PresetPositions {
    public enum PositionName {INTAKE, DEPOSIT_FRONT_TOP, SAFE_DRIVE}
    public double pitch;
    public double roll;
    public double elbowAngle;
    public double slideLength;
    public PositionName name;
    public PresetPositions(PositionName name, double pitch, double roll, double elbowAngle, double slideLength){
        this.name = name;
        this.pitch = pitch;
        this.roll = roll;
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }

    public String toString(){
        return name.toString() + "," + pitch + "," + roll + "," + elbowAngle + "," + slideLength;
    }
    public PositionName getName(){
        return name;
    }

    public static PresetPositions INTAKE_POSITION = new PresetPositions(PositionName.INTAKE, -30, -90, 0 ,2);
    public static PresetPositions DEPOSIT_FRONT_TOP_POSITION = new PresetPositions(PositionName.DEPOSIT_FRONT_TOP, 50, -90, 70, 28.5);
    public static PresetPositions SAFE_DRIVING_POSITION = new PresetPositions(PositionName.SAFE_DRIVE, -90,-90,58, 0);
}
