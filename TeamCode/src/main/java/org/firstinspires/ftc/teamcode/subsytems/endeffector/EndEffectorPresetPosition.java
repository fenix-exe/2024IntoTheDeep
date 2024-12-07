package org.firstinspires.ftc.teamcode.subsytems.endeffector;


public class EndEffectorPresetPosition {

    EndEffectorPresetPositionNames name;
    double pitch;
    double roll;
    public EndEffectorPresetPosition(EndEffectorPresetPositionNames name, double pitch, double roll){
        this.name = name;
        this.pitch = pitch;
        this.roll = roll;
    }
    public EndEffectorPresetPositionNames getName() {
        return this.name;
    }

    public String toString(){
        return this. name+","+this.pitch+","+this.roll;
    }

    public static EndEffectorPresetPosition INTAKE_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.INTAKE,-30,-90);
    public static EndEffectorPresetPosition DEPOSIT_FRONT_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DEPOSIT_FRONT_TOP, 50,-90);
    public static EndEffectorPresetPosition DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DEPOSIT_FRONT_BOTTOM, 50,-90);
    public static EndEffectorPresetPosition DEPOSIT_BACK_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DEPOSIT_BACK_TOP, -90,90);
    public static EndEffectorPresetPosition DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DEPOSIT_BACK_BOTTOM,-90,90);
    public static EndEffectorPresetPosition SAFE_DRIVING_POSITION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DRIVING,-90,-90);
    public static EndEffectorPresetPosition DISPERSION = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.DISPERSION,25,-90);
    public static EndEffectorPresetPosition INTAKE_DOWN = new EndEffectorPresetPosition(EndEffectorPresetPositionNames.INTAKE_DOWN,-22, -90);
}