package org.firstinspires.ftc.teamcode.subsytems.arm;

public class MoveElbowToPresetAction {
    double targetAngle;
    Arm arm;
    public MoveElbowToPresetAction(Arm arm, double targetAngle){
        this.targetAngle = targetAngle;
        this.arm=arm;
    }
    public void execute(){
        arm.setElbowAngleInDegrees(targetAngle);
    }

}
