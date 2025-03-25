package org.firstinspires.ftc.teamcode.teleop.subsytems.intake;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

public class BigWheelIntake implements IIntake{
    CRServoImplEx leftRoller;
    CRServoImplEx rightRoller;
    public BigWheelIntake(CRServoImplEx leftRoller, CRServoImplEx rightRoller){
        this.leftRoller=leftRoller;
        this.rightRoller=rightRoller;
    }
    @Override
    public void intake() {
        leftRoller.setPower(RobotConstants.INTAKE_SPEED);
        rightRoller.setPower(RobotConstants.INTAKE_SPEED);
    }

    @Override
    public void outtake() {
        leftRoller.setPower(RobotConstants.OUTTAKE_SPEED);
        rightRoller.setPower(RobotConstants.OUTTAKE_SPEED);
    }

    @Override
    public void stop() {
        leftRoller.setPower(RobotConstants.STOP_SPEED);
        rightRoller.setPower(RobotConstants.STOP_SPEED);
    }
}
