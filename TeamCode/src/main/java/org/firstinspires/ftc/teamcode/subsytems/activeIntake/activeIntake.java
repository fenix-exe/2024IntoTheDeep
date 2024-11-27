package org.firstinspires.ftc.teamcode.subsytems.activeIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

import androidx.annotation.NonNull;

public class activeIntake {
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
        intake.setPower(-1);
        setIntakePos(intakeState.FORWARD);
    }

    public void intakeOff() {
        intake.setPower(0);
        setIntakePos(intakeState.OFF);
    }

    public void intakeBack() {
        intake.setPower(0.5);
        setIntakePos(intakeState.BACKWARD);
    }

    public class aIControl implements Action {
        private final double targetPower;

        public aIControl(double targetPower) {
            this.targetPower = targetPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(targetPower);
            return false;
        }
    }

    public Action aIControl(double targetPower) {
        return new aIControl(targetPower);
    }


}
