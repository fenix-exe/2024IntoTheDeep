package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

public class PinchRollerIntake {
    protected CRServoImplEx intake;
    public PinchRollerIntake(CRServoImplEx intake){
        this.intake = intake;
    }
    public void intake(){
        intake.setPower(RobotConstants.BARKER_INTAKE_SPEED);
    }
    public void outtake(){
        intake.setPower(RobotConstants.BARKER_OUTTAKE_SPEED);
    }
    public void stop(){
        intake.setPower(RobotConstants.BARKER_STOP_SPEED);

    }
}
