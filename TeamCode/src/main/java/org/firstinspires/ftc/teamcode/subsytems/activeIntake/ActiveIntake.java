package org.firstinspires.ftc.teamcode.subsytems.activeIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import androidx.annotation.NonNull;

public class ActiveIntake {
    public CRServo intakeServo;
    RevColorSensorV3 colorSensor;

    public enum intakeState {FORWARD,BACKWARD,OFF}

    public intakeState intakePos;

    public void setIntakePos(intakeState intakePos) {
        this.intakePos = intakePos;
    }

    public ActiveIntake(CRServo intake, RevColorSensorV3 colorSensor){
        this.intakeServo =intake;
        this.colorSensor = colorSensor;
    }

    public void intakeForward() {
        intakeServo.setPower(-1);
        setIntakePos(intakeState.FORWARD);
    }

    public void intakeOff() {
        intakeServo.setPower(0);
        setIntakePos(intakeState.OFF);
    }

    public void intakeBack() {
        intakeServo.setPower(0.5);
        setIntakePos(intakeState.BACKWARD);
    }
    public boolean blockIn(){
        return colorSensor.getDistance(DistanceUnit.MM) < 20;
    }





}
