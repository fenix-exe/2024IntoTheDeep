package org.firstinspires.ftc.teamcode.subsytems.pivot;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import androidx.annotation.NonNull;

import static java.lang.Math.floor;


public class pivotCodeFunctions {
    DcMotorEx pivot;
    int pivotPos;
    int topPos;
    PivotPIDFFunctions functions;

    public pivotCodeFunctions(DcMotorEx pivot, PivotPIDFFunctions functions, int topPos) {
        this.pivot = pivot;
        this.functions = functions;
        this.topPos = topPos;
    }

    public void goTo(int targetPos) {
        pivotPos = targetPos;
        if (pivotPos < -15){
            pivotPos = -15;
        }
        if (pivotPos > topPos) {
            pivotPos = topPos;
        }

        pivot.setPower(1);
        pivot.setTargetPosition(pivotPos);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pivotJoystick(int startPos, double pivotControlJoystick) {
        /*if (pivotControlJoystick != 0) {
            pivotPos = (int) floor(startPos + pivotControlJoystick * 400);
        }
        goTo(pivotPos);*/
        double power;
        if (pivot.getCurrentPosition() > topPos - 100 && pivotControlJoystick > 0){
            power = 0;
        } else if (pivot.getCurrentPosition() < 100 && pivotControlJoystick < 0) {
            power = 0;
        } else {
            power = pivotControlJoystick * 0.5;
        }
        pivot.setPower(power);
    }

    public void setNewTopPos(int topPos) {
        this.topPos = topPos;
    }

    public double ticksToDegrees(int ticks) {
        return ticks / 24.22;
    }

    public int degreesToTicks(double degrees) {
        return (int) floor(degrees * 24.22);
    }

    public double getElbowAngle() {
        return ticksToDegrees(pivot.getCurrentPosition());
    }

    public void setElbowAngle(double angle){
        int ticks = degreesToTicks(angle);
        goTo(ticks);
    }
    public int getElbowTicks(){
        return pivot.getCurrentPosition();
    }

    public boolean isBusy() {
        return pivot.isBusy();
    }


    public class elbowControl implements Action {
        private final int target;

        elbowControl(int targetPos) {
            this.target = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goTo(target);
            if (target-30 < getElbowTicks() && getElbowTicks() < target+30) {
                pivot.setPower(0);
                return false;
            } else {
                return true;
            }
        }


    }
    public Action elbowControl(int targetPos) {
        return new elbowControl(targetPos);
    }
}
