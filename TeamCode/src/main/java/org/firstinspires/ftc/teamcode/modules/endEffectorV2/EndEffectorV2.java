package org.firstinspires.ftc.teamcode.modules.endEffectorV2;

import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffector;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class EndEffectorV2 {
    Wrist wrist;
    Claw claw;
    public EndEffectorV2(Wrist wrist, Claw claw){
        this.wrist = wrist;
        this.claw = claw;
    }

    public void manualPitch(double stepSize, double initialPosition){
        wrist.manualControlPitch(stepSize, initialPosition);
    }
    public void manualRoll(double stepSize, double initialPosition){
        wrist.manualControlRoll(stepSize, initialPosition);
    }
    public void goToPresetPosition (double pitch, double roll){
        wrist.presetPositionPitch(pitch);
        wrist.presetPositionRoll(roll);
    }
    public void openClaw(){
        claw.openClaw();
    }
    public void closeClaw(){
        claw.closeClaw();
    }

}
