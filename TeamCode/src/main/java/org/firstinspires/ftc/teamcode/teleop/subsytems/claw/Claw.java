package org.firstinspires.ftc.teamcode.teleop.subsytems.claw;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

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
