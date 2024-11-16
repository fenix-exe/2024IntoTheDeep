package org.firstinspires.ftc.teamcode.subsytems.arm;

public class ArmPresetPosition {
    double slideLength;
    int elbowAngle;
    public ArmPresetPosition(int elbowAngle, double slideLength){
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }

    public static ArmPresetPosition INTAKE_POSITION = new ArmPresetPosition(0,0);
    public static ArmPresetPosition DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(70,30);
    public static ArmPresetPosition DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(70,12.75);
    public static ArmPresetPosition DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(88,25);
    public static ArmPresetPosition DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(88,7.75);

    public static ArmPresetPosition SAFE_DRIVING_POSITION = new ArmPresetPosition(45,0);
}
