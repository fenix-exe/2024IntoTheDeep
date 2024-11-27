package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import androidx.annotation.NonNull;

public class ActiveIntake {
    CRServo intake;
    RevColorSensorV3 colorSensor;

    public enum intakeState {FORWARD,BACKWARD,OFF}

    public intakeState intakePos;
    public intakeState getIntakePos() {
        return intakePos;
    }

    public void setIntakePos(intakeState intakePos) {
        this.intakePos = intakePos;
    }

    public ActiveIntake(CRServo intake, RevColorSensorV3 colorSensor){
        this.intake=intake;
        this.colorSensor = colorSensor;
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
    public boolean blockIn(){
        return colorSensor.getDistance(DistanceUnit.MM) < 20;
    }

    public class aIControl implements Action {
        private final double targetPower;

        public aIControl(double targetPower) {
            this.targetPower = targetPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPower() != targetPower) {
                intake.setPower(targetPower);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action aIControl(double targetPower) {
        return new aIControl(targetPower);
    }

    public class intakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (true) {
                //TODO: add color sensor code
                intakeForward();
                return true;
            } else {
                intakeOff();
                return false;
            }
        }
    }

    public Action intakeIn() {
        return new intakeIn();
    }

    public class intakeOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (true) {
                //TODO: add color sensor code
                intakeBack();
                return true;
            } else {
                intakeOff();
                return false;
            }
        }
    }

    public Action intakeOut() {
        return new intakeOut();
    }



}
