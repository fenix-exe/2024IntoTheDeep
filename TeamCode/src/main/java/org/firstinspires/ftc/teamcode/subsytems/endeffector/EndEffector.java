package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

public class EndEffector {
    ActiveIntake intake;
    Differential differential;

    public EndEffector(ActiveIntake intake, Differential differential ){
        this.intake = intake;
        this.differential = differential;
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
    public void setDifferentialPosition(double pitch, double roll){
        differential.setDifferentialPosition(pitch, roll);
    }

}
