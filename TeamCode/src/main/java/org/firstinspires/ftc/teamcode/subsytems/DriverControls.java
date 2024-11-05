package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriverControls implements DriveControlMap{
    Gamepad gamepad1;
    Gamepad gamepad2;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    public DriverControls(Gamepad gamepad1, Gamepad gamepad2, Gamepad gamepad1previous, Gamepad gamepad2previous){
        this.gamepad1=gamepad1;
        this.gamepad2 = gamepad2;
        this.gamepad1previous = gamepad1previous;
        this.gamepad2previous= gamepad2previous;
    }
    @Override
    public boolean slowMode() {
        return gamepad1.left_bumper && !gamepad1previous.left_bumper;
    }

    @Override
    public boolean driveTypeSwitch() {
        return gamepad1.right_bumper && !gamepad1previous.right_bumper;
    }

    @Override
    public boolean resetIMU() {
        return gamepad1.a;
    }

    @Override
    public boolean emergencyStop() {
        return gamepad1.start;
    }

    @Override
    public boolean undoEmergencyStop() {
        return gamepad1.options;
    }

    @Override
    public boolean slidesFullyUp() {
        return gamepad2.dpad_up;
        //return gamepad1.dpad_up;
    }

    @Override
    public boolean slidesFullyDown() {
        return gamepad2.dpad_down;
        //return gamepad1.dpad_down;
    }

    @Override
    public boolean pivotParallel() {
        //return gamepad2.b
        return gamepad1.b;
    }

    @Override
    public boolean pivotPerp() {
        //return gamepad2.y
        return gamepad1.y;
    }

    @Override
    public double slideMovement() {
        return -gamepad2.left_stick_y;
        //return -gamepad1.right_stick_y;
    }

    @Override
    public double degreeOfFreedomX() {
        return gamepad2.right_stick_x;
    }

    @Override
    public double degreeOfFreedomY() {
        return -gamepad2.right_stick_y;
    }

    @Override
    public boolean intakePower() {
        return gamepad2.left_bumper && !gamepad2previous.left_bumper;
    }

    @Override
    public boolean intakeDirection() {
        return gamepad2.right_bumper && !gamepad2previous.right_bumper;
    }

    @Override
    public float intakenewForward() {
        return gamepad2.left_trigger;
    }

    @Override
    public float intakenewBackward() {
        return gamepad2.right_trigger;
    }

    public boolean resetWrist() {
        return gamepad2.a;
    }
}
