package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

public class EndEffector {
    ActiveIntake intake;
    differential diffy;
    double STEP_SIZE = 0.5;

    public EndEffector(ActiveIntake intake, differential diffy ){
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
        double pitch = controlPitch*STEP_SIZE + diffy.returnPitch();
        double roll = controlRoll*STEP_SIZE + diffy.returnRoll();
        setDifferentialPosition(pitch, roll);
    }

}
