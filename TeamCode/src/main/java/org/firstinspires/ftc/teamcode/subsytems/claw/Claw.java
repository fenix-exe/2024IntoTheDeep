package org.firstinspires.ftc.teamcode.subsytems.claw;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    double OPEN_POSITION = 0.5;
    double CLOSED_POSITION = 1;
    public Claw (Servo claw){
        this.claw = claw;
    }
    public void openClaw(){
        claw.setPosition(OPEN_POSITION);
    }
    public void closeClaw(){
        claw.setPosition(CLOSED_POSITION);
    }
}
