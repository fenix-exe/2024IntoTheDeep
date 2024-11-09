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
        if (pivotPos < 10){
            pivotPos = 10;
        }
        if (pivotPos > topPos - 10){
            pivotPos = topPos;
        }
        /*if (pivot.getCurrentPosition() > pivotPos) {
            pivot.setPower(-1);
        } else {
            pivot.setPower(1);
        }*/
        pivot.setPower(0.8);
        pivot.setTargetPosition(pivotPos);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void pivotJoystick(int startPos, double pivotControlJoystick){
        if (pivotControlJoystick != 0) {
            pivotPos = (int) floor(startPos + pivotControlJoystick * 50);
        }
        goTo(pivotPos);
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
