package org.firstinspires.ftc.teamcode.teleop.subsytems.intake;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;

public class BigWheelIntake implements IIntake{

    private IntakeDirection intakeStates;

    protected CRServoImplEx leftRoller;
    protected CRServoImplEx rightRoller;

    public BigWheelIntake(CRServoImplEx leftRoller, CRServoImplEx rightRoller){
        this.leftRoller=leftRoller;
        this.rightRoller=rightRoller;
        intakeStates = IntakeDirection.OFF;
    }
    @Override
    public void intake() {
        leftRoller.setPower(RobotConstants.INTAKE_SPEED);
        rightRoller.setPower(RobotConstants.INTAKE_SPEED);
        intakeStates = IntakeDirection.FORWARD;
    }

    @Override
    public void outtake() {
        leftRoller.setPower(RobotConstants.OUTTAKE_SPEED);
        rightRoller.setPower(RobotConstants.OUTTAKE_SPEED);
        intakeStates = IntakeDirection.BACKWARD;
    }

    @Override
    public void slowOuttake() {
        leftRoller.setPower(RobotConstants.SLOW_OUTTAKE_SPEED);
        rightRoller.setPower(RobotConstants.SLOW_OUTTAKE_SPEED);
        intakeStates = IntakeDirection.BACKWARD;
    }
    @Override
    public void stop() {
        leftRoller.setPower(RobotConstants.STOP_SPEED);
        rightRoller.setPower(RobotConstants.STOP_SPEED);
        intakeStates = IntakeDirection.OFF;
    }

    @Override
    public IntakeDirection getIntakeDirection() {
        return intakeStates;
    }
}
