package org.firstinspires.ftc.teamcode.subsytems.arm;

public class ArmPresetPosition {
    double slideLength;
    double elbowAngle;
    public ArmPresetPosition(double elbowAngle, double slideLength){
        this.elbowAngle = elbowAngle;
        this.slideLength = slideLength;
    }

    public static ArmPresetPosition INTAKE_POSITION = new ArmPresetPosition(0,5);
    public static ArmPresetPosition DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(70,28.5);
    public static ArmPresetPosition DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(70,11.25);
    public static ArmPresetPosition DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(90,25);
    public static ArmPresetPosition DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(90,7.75);

    public static ArmPresetPosition SAFE_DRIVING_POSITION = new ArmPresetPosition(65,0);
    public static ArmPresetPosition ASCENT_2_HANG = new ArmPresetPosition(90, 0);
    public static ArmPresetPosition FLAT_ELBOW = new ArmPresetPosition(0,0);
}
