package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriverControls implements DriveControlMap{
    Gamepad gamepad1current;
    Gamepad gamepad2current;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad physicalGamepad1;
    Gamepad physicalGamepad2;
    public DriverControls(Gamepad gamepad1, Gamepad gamepad2, Gamepad gamepad1previous, Gamepad gamepad2previous){
        this.gamepad1current=gamepad1;
        this.gamepad2current = gamepad2;
        this.gamepad1previous = gamepad1previous;
        this.gamepad2previous= gamepad2previous;
    }
    public DriverControls (Gamepad gamepad1, Gamepad gamepad2){
        this.physicalGamepad1 = gamepad1;
        this.physicalGamepad2 = gamepad2;

        gamepad1current = new Gamepad();
        gamepad2current = new Gamepad();

        gamepad1previous = new Gamepad();
        gamepad2previous = new Gamepad();

        gamepad1current.copy(gamepad1);
        gamepad2current.copy(gamepad2);
    }

    public void update(){
            gamepad1previous.copy(gamepad1current);
            gamepad2previous.copy(gamepad2current);

            gamepad1current.copy(physicalGamepad1);
            gamepad2current.copy(physicalGamepad2);

    }

    @Override
    public boolean slowMode() {
        return gamepad1current.left_bumper && !gamepad1previous.left_bumper;
    }

    @Override
    public boolean driveTypeSwitch() {
        return gamepad1current.right_bumper && !gamepad1previous.right_bumper;
    }

    @Override
    public boolean resetIMU() {
        return gamepad1current.a;
    }

    @Override
    public boolean emergencyStop() {
        return gamepad1current.start;
    }

    @Override
    public boolean undoEmergencyStop() {
        return gamepad1current.options;
    }

    @Override
    public boolean slidesFullyUp() {
        return gamepad2current.dpad_up;
        //return gamepad1.dpad_up;
    }

    @Override
    public boolean slidesFullyDown() {
        return gamepad2current.dpad_down;
        //return gamepad1.dpad_down;
    }

    @Override
    public boolean pivotParallel() {
        //return gamepad2current.b
        return gamepad2current.b;
    }

    @Override
    public boolean pivotPerp() {
        //return gamepad2.y
        return gamepad2current.y;
    }

    @Override
    public double pivotJoystick() {
        return gamepad2current.left_stick_x;
    }

    @Override
    public double slideMovement() {
        return -gamepad2current.left_stick_y;
        //return -gamepad1.right_stick_y;
    }

    @Override
    public double degreeOfFreedomX() {
        return gamepad2current.right_stick_x;
    }

    @Override
    public double degreeOfFreedomY() {
        return -gamepad2current.right_stick_y;
    }

    @Override
    public boolean intakePower() {
        return gamepad2current.left_bumper && !gamepad2previous.left_bumper;
    }

    @Override
    public boolean intakeDirection() {
        return gamepad2current.right_bumper && !gamepad2previous.right_bumper;
    }

    @Override
    public float intakenewForward() {
        return gamepad2current.left_trigger;
    }

    @Override
    public float intakenewBackward() {
        return gamepad2current.right_trigger;
    }

    @Override
    public boolean submersibleIntakeReady() {
        return gamepad2current.a;
    }

    @Override
    public boolean acsent1Park() {
        return gamepad2current.start;
    }

    @Override
    public boolean drivingPos() {
        return false;
    }

    @Override
    public boolean depositReadyBack() {
        return gamepad2current.left_bumper;
    }

    @Override
    public boolean depositReadyUp() {
        return gamepad2current.right_bumper;
    }
}
