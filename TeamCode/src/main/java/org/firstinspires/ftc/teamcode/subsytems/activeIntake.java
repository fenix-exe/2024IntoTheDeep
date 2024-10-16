package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

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


}
