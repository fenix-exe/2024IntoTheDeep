package org.firstinspires.ftc.teamcode.modules.arm;

public class ArmPresetPosition {

    ArmPresetPositionNames name;
    double slideLength;
    double elbowAngle;
    public ArmPresetPosition(ArmPresetPositionNames name, double elbowAngle, double slideLength){
        this.name = name;
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }

    public ArmPresetPositionNames getName() {
        return this.name;
    }

    public String toString(){
        return this.name.toString()+","+this.elbowAngle+","+this.slideLength;
    }


    public static ArmPresetPosition INTAKE_POSITION = new ArmPresetPosition(ArmPresetPositionNames.INTAKE,2,5);
    public static ArmPresetPosition DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_FRONT_TOP,70,28.5);
    public static ArmPresetPosition DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_FRONT_BOTTOM,70,11.25);
    public static ArmPresetPosition DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_BACK_TOP,90,25);
    public static ArmPresetPosition DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DEPOSIT_BACK_BOTTOM,90,7.75);

    public static ArmPresetPosition SAFE_DRIVING_POSITION = new ArmPresetPosition(ArmPresetPositionNames.DRIVING,65,0);
    public static ArmPresetPosition ASCENT_2_HANG = new ArmPresetPosition(ArmPresetPositionNames.ASCENT_2_HANG, 90, 0);
    public static ArmPresetPosition FLAT_ELBOW = new ArmPresetPosition(ArmPresetPositionNames.FLAT_ELBOW,0,0);
    public static ArmPresetPosition INTAKE_DOWN = new ArmPresetPosition(ArmPresetPositionNames.INTAKE_DOWN,-6, 4);
}
