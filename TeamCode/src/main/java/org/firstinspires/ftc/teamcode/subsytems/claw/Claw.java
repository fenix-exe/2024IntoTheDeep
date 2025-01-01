package org.firstinspires.ftc.teamcode.subsytems.claw;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

public class Claw {
    Servo claw;

    public Claw (Servo claw){
        this.claw = claw;
    }
    public void openClaw(){
        claw.setPosition(RobotConstants.OPEN_POSITION);
    }
    public void closeClaw(){claw.setPosition(RobotConstants.CLOSED_POSITION);}
    public void intermediateClaw(){claw.setPosition(RobotConstants.INTERMEDIATE_POSITION);}
    public double getClawPosition(){
        return claw.getPosition();
    }
}
