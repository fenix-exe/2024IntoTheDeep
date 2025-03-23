package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class CommonLinearActuator {
    protected double ENCODER_RES = 103.8;
    protected double LEAD_MM = 8;
    protected double MM_TO_INCHES = 1/25.4;
    DcMotorEx linearActuatorMotor;
    RevTouchSensor limitSwitch;
    public CommonLinearActuator(DcMotorEx linearActuatorMotor, RevTouchSensor limitSwitch){
        this.linearActuatorMotor = linearActuatorMotor;
        this.limitSwitch = limitSwitch;
    }
    public double ticksToInches(int ticks){
        return (ticks/ ENCODER_RES)*(LEAD_MM * MM_TO_INCHES);
    }
    public int inchesToTicks(double inches){
        return (int) (Math.floor(((inches/MM_TO_INCHES)/LEAD_MM)) * ENCODER_RES);
    }
    public double getLinearActuatorPositionInches(){
        return ticksToInches(linearActuatorMotor.getCurrentPosition());
    }
    public double getLinearActuatorTargetPositionInches(){
        return ticksToInches(linearActuatorMotor.getTargetPosition());
    }
    public void goToTargetPositionInches(double position){
        int positionInTicks = inchesToTicks(position);
        linearActuatorMotor.setTargetPosition(positionInTicks);
        linearActuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuatorMotor.setPower(1);
    }
    public void resetEncoders(){
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean getLimitSwitchState(){
        return limitSwitch.isPressed();
    }
    public void setLinearActuatorPower(double power){
        linearActuatorMotor.setPower(power);
    }
}
