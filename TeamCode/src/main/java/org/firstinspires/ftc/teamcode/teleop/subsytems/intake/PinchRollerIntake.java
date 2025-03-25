package org.firstinspires.ftc.teamcode.teleop.subsytems.intake;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

public class PinchRollerIntake implements IIntake{
    CRServoImplEx rollers;
    public PinchRollerIntake(CRServoImplEx rollers){
        this.rollers=rollers;
    }
    @Override
    public void intake() {
        rollers.setPower(RobotConstants.INTAKE_SPEED);
    }

    @Override
    public void outtake() {
        rollers.setPower(RobotConstants.OUTTAKE_SPEED);
    }

    @Override
    public void stop() {
        rollers.setPower(RobotConstants.STOP_SPEED);
    }
}
