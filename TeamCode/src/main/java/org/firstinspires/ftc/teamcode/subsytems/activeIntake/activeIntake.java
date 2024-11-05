package org.firstinspires.ftc.teamcode.subsytems.activeIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import androidx.annotation.NonNull;

public class activeIntake {
    Gamepad gamepad2;
    Gamepad gamepad2previous;
    CRServo intake;

    public enum intakeState {FORWARD,BACKWARD,OFF}

    public intakeState intakePos;
    public intakeState getIntakePos() {
        return intakePos;
    }

    public void setIntakePos(intakeState intakePos) {
        this.intakePos = intakePos;
    }

    public activeIntake(CRServo intake){
        this.intake=intake;
    }

    public void intakeForward() {
        intake.setPower(1);
        setIntakePos(intakeState.FORWARD);
    }

    public void intakeOff() {
        intake.setPower(0);
        setIntakePos(intakeState.OFF);
    }

    public void intakeBack() {
        intake.setPower(-1);
        setIntakePos(intakeState.BACKWARD);
    }

    public class aIForward implements Action {

        double intakePos = intake.getPower();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intakePos != 1) {
                intake.setPower(1);
                return false;
            }
            else if (intakePos == 1) {
                return true;
            }
            return false;
        }
    }

    public class aIStop implements Action {

        double intakePos = intake.getPower();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intakePos != 0) {
                intake.setPower(0);
                return false;
            }
            else if (intakePos == 0) {
                return true;
            }
            return false;
        }
    }

    public class aIBackward implements Action {

        double intakePos = intake.getPower();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intakePos != -1) {
                intake.setPower(-1);
                return false;
            }
            else if (intakePos == -1) {
                return true;
            }
            return false;
        }
    }

    public Action aIForward() {
        return new aIForward();
    }

    public Action aIStop() {
        return new aIStop();
    }

    public Action aIBackward() {
        return new aIBackward();
    }




}
