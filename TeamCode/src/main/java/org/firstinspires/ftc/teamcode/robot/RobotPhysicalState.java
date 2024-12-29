package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainSpeedMultiplier;

public class RobotPhysicalState {
    public double slidePositionInInches;
    public double elbowAngleInDegrees;
    public double differentialRollInDegrees;
    public double differentialPitchInDegrees;
    public boolean intakeActive;
    public RobotPhysicalState(){

    }
    public double getSpeedMultiplier(){
        if (elbowAngleInDegrees > 45){
            if (slidePositionInInches > 10){
                return DriveTrainSpeedMultiplier.SUPER_SLOW;
            } else {
                return DriveTrainSpeedMultiplier.HALF_SPEED;
            }
        }
        return DriveTrainSpeedMultiplier.NO_MULTIPLIER;

    }

}
