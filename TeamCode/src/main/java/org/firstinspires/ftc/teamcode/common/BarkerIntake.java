package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

public class BarkerIntake {
    private CRServo intake;
    public BarkerIntake(CRServo intake){
        this.intake = intake;
    }
    public void intake(){
        intake.setPower(RobotConstants.BARKER_INTAKE_SPEED);
    }
    public void outtake(){
        intake.setPower(RobotConstants.BARKER_OUTTAKE_SPEED);
    }
    public void stop(){
        intake.setPower(0);
    }
    public void setPower(double power){
        intake.setPower(power);
    }
}
