package org.firstinspires.ftc.teamcode.subsytems.pivot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.floor;


public class pivotCodeFunctions {
    DcMotorEx pivot;
    int pivotPos;
    int topPos;
    PivotPIDFFunctions functions;
    public pivotCodeFunctions(DcMotorEx pivot,PivotPIDFFunctions functions, int topPos){
        this.pivot=pivot;
        this.functions=functions;
        this.topPos = topPos;
    }
    public void goTo(int targetPos){
        pivotPos = targetPos;
        if (pivotPos < 0){
            pivotPos = 0;
        }
        if (pivotPos > topPos){
            pivotPos = topPos;
        }
        /*if (pivot.getCurrentPosition() > pivotPos) {
            pivot.setPower(-1);
        } else {
            pivot.setPower(1);
        }*/
        pivot.setPower(1);
        pivot.setTargetPosition(pivotPos);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void pivotJoystick(int startPos, double pivotControlJoystick){
        /*if (pivotControlJoystick != 0) {
            pivotPos = (int) floor(startPos + pivotControlJoystick * 400);
        }
        goTo(pivotPos);*/
        double power;



        if (pivot.getCurrentPosition() > topPos - 100 && pivotControlJoystick > 0){
            power = 0;
        } else if (pivot.getCurrentPosition() < 100 && pivotControlJoystick < 0){
            power = 0;
        } else {
            power = pivotControlJoystick*0.5;
        }
        pivot.setPower(power);
    }

    public void setNewTopPos(int topPos){
        this.topPos = topPos;
    }
    public double ticksToDegrees(int ticks){
        return ticks/24.22;
    }
    public int degreesToTicks(double degrees){
        return (int) floor(degrees * 24.22);
    }
    public double getElbowAngle(){
        return ticksToDegrees(pivot.getCurrentPosition());
    }
    public int getElbowTicks(){
        return pivot.getCurrentPosition();
    }




}
