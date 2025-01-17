package org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LinearActuator {
    protected double ENCODER_RES = 103.8;
    protected double LEAD_MM = 8;
    protected double MM_TO_INCHES = 1/25.4;
    DcMotorEx linearActuatorMotor;
    public LinearActuator(DcMotorEx linearActuatorMotor){
        this.linearActuatorMotor = linearActuatorMotor;
    }
    private double ticksToInches(int ticks){
        return (ticks/ ENCODER_RES)*(LEAD_MM * MM_TO_INCHES);
    }
    private int inchesToTicks(double inches){
        return (int) (Math.floor(((inches/MM_TO_INCHES)/LEAD_MM)) * ENCODER_RES);
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
}
