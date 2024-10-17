package org.firstinspires.ftc.teamcode.subsytems.pivot;


import com.qualcomm.robotcore.hardware.DcMotorEx;

public class pivotCodeFunctions {
    DcMotorEx pivot;
    int pivotPos;
    PivotPIDFFunctions functions;
    public pivotCodeFunctions(DcMotorEx pivot,PivotPIDFFunctions functions){
        this.pivot=pivot;
        this.functions=functions;
    }
    public void goTo(int targetPos){
        pivotPos = targetPos;
        if (pivotPos < 0){
            pivotPos = 0;
        }
        if (pivotPos > 2187){
            pivotPos = 2187;
        }
        functions.moveToPos(pivot.getCurrentPosition(), targetPos);
    }
    

}
