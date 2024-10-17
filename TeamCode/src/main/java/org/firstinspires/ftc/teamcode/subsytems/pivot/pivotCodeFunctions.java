package org.firstinspires.ftc.teamcode.subsytems.pivot;


import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        functions.moveToPos(pivot.getCurrentPosition(), targetPos);
    }
    

}
