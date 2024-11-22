package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.subsytems.arm.ArmPresetPosition;

public class EndEffectorPresetPosition {

    double pitch;
    double roll;
    public EndEffectorPresetPosition(double pitch, double roll){
        this.pitch = pitch;
        this.roll = roll;
    }

    public static EndEffectorPresetPosition INTAKE_POSITION = new EndEffectorPresetPosition(0,-90);
    public static EndEffectorPresetPosition DEPOSIT_FRONT_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(50,-90);
    public static EndEffectorPresetPosition DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(50,-90);
    public static EndEffectorPresetPosition DEPOSIT_BACK_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(-90,90);
    public static EndEffectorPresetPosition DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(-90,90);
    public static EndEffectorPresetPosition SAFE_DRIVING_POSITION = new EndEffectorPresetPosition(-90,-90);
    public static EndEffectorPresetPosition INTAKE_DOWN_ON_GROUND = new EndEffectorPresetPosition(25,-90);
}