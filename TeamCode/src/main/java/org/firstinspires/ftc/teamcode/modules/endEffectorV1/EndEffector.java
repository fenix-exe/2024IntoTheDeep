package org.firstinspires.ftc.teamcode.modules.endEffectorV1;

import org.firstinspires.ftc.teamcode.subsytems.differential.Differential;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.ActiveIntake;

import java.util.HashMap;

public class EndEffector {
    ActiveIntake intake;
    Differential diffy;

    public EndEffector(ActiveIntake intake, Differential diffy ){
        this.intake = intake;
        this.diffy = diffy;
    }
    public void activeIntakeForward(){
        intake.intakeForward();
    }
    public void activeIntakeBackward(){
        intake.intakeBack();
    }
    public void activeIntakeOff(){
        intake.intakeOff();
    }
    public boolean blockIn(){return intake.blockIn();}
    public void setDifferentialPosition(double pitch, double roll){
        diffy.setDifferentialPosition(pitch, roll);
    }
    public void diffyJoystick(double controlPitch, double controlRoll){
        double pitch = controlPitch* EndEffectorConstants.STEP_SIZE + diffy.returnPitch();
        double roll = controlRoll* EndEffectorConstants.STEP_SIZE + diffy.returnRoll();
        setDifferentialPosition(pitch, roll);
    }
    public HashMap getDebugInfo() {
        /*telemetry.addData("Slide extension", arm.getSlideExtension());
        telemetry.addData("Slide target position", arm.getSlideExtension());
        telemetry.addData("Slide limit", arm.getSlideMaxLengthIn42Inches(arm.getElbowAngleInTicks()));
        telemetry.addData("Elbow angle", arm.getElbowAngleInDegrees());
        telemetry.addData("Elbow target position", pivot.getTargetPosition());*/

        HashMap debugInfo = new HashMap<>();
        debugInfo.put("Differential Pitch", diffy.returnPitch());
        debugInfo.put("Differential Roll", diffy.returnRoll());
        debugInfo.put("Active Intake Power", intake.intakeServo.getPower());
        return debugInfo;
    }

}
