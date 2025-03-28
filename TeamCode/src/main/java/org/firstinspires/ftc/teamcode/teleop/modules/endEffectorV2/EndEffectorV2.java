package org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2;

import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

import java.util.HashMap;

public class EndEffectorV2 {
    Wrist wrist;
    Claw claw;
    public EndEffectorV2(Wrist wrist, Claw claw){
        this.wrist = wrist;
        this.claw = claw;
    }

    public void manualPitch(double stepSize, double initialPosition){
        wrist.manualControlPitch(stepSize);
    }
    public void presetPitch(double pitch){
        wrist.presetPositionPitch(pitch);
    }
    public void openClaw(){
        claw.openClaw();
    }
    public void closeClaw(){
        claw.closeClaw();
    }
    public HashMap getDebugInfo(){
        HashMap debugMap = new HashMap();
        debugMap.put("Pitch Angle", String.valueOf(wrist.getPitchAngle()));
        debugMap.put("Claw Servo Position", String.valueOf(claw.getClawPosition()));
        return debugMap;
    }

}
