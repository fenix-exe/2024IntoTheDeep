package org.firstinspires.ftc.teamcode.teleop.modules.driverControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashSet;
import java.util.Set;

public class DriverControls implements DriveControlMap {
    public enum scoringType {SAMPLE, SPECIMEN}
    scoringType gameStrategyMode;
    Gamepad gamepad1current;
    Gamepad gamepad2current;
    Gamepad gamepad1previous;
    Gamepad gamepad2previous;
    Gamepad physicalGamepad1;
    Gamepad physicalGamepad2;

    double y;
    public DriverControls (Gamepad gamepad1, Gamepad gamepad2, double y){
        this.physicalGamepad1 = gamepad1;
        this.physicalGamepad2 = gamepad2;
        this.y = y;

        gamepad1current = new Gamepad();
        gamepad2current = new Gamepad();

        gamepad1previous = new Gamepad();
        gamepad2previous = new Gamepad();

        gamepad1current.copy(gamepad1);
        gamepad2current.copy(gamepad2);

        gameStrategyMode = scoringType.SAMPLE;
    }
    @Override
    public void update(){
            gamepad1previous.copy(gamepad1current);
            gamepad2previous.copy(gamepad2current);

            gamepad1current.copy(physicalGamepad1);
            gamepad2current.copy(physicalGamepad2);

    }
    public double gamepadStickValue(double stickValue){
        return Math.pow(stickValue, y);
    }
    @Override
    public double forwardDrive(){
        if (Math.abs(gamepad1current.left_stick_y) > 0.3){
            return -gamepad1current.left_stick_y;
        }
        return 0;
    }
    @Override
    public double strafeDrive(){
        if (Math.abs(gamepad1current.left_stick_x) > 0.3){
            return gamepad1current.left_stick_x;
        }
        return 0;
    }
    @Override
    public double turnDrive(){
        if (Math.abs(gamepad1current.right_stick_x) > 0.3){
            return gamepad1current.right_stick_x;
        }
        return 0;
    }

    @Override
    public boolean driveTypeSwitch() {
        return gamepad1current.dpad_down && !gamepad1previous.dpad_down;
    }

    @Override
    public boolean resetIMU() {
        return gamepad1current.dpad_up;
    }
    @Override
    public boolean slowMode(){
        return gamepad1current.right_bumper;
    }

    @Override
    public double pivotJoystick() {
        if (Math.abs(gamepad2current.right_stick_y) > 0.5){ // setting the threshold at which we want to set a positive or negative value
            return -gamepad2current.right_stick_y;
        }
        return 0;
    }
    @Override
    public boolean pivotManualStopped(){
        return Math.abs(gamepad2previous.left_stick_y) > 0.65 && !(Math.abs(gamepad2current.left_stick_y) > 0.65);
    }

    @Override
    public double slideMovement() {
        if (Math.abs(gamepad2current.left_stick_y) > 0.65){
            return -gamepad2current.left_stick_y;
        }
        if (gamepad1current.right_trigger > 0.3){
            return gamepad1current.right_trigger * 0.5;
        }
        if (gamepad1current.left_trigger > 0.3){
            return -gamepad1current.left_trigger * 0.5;
        }
        return 0;
    }
    @Override
    public boolean slideStopped() {
        if (Math.abs(gamepad2previous.right_stick_y) > 0.5 && !(Math.abs(gamepad2current.right_stick_y) > 0.5)){
            return true;
        }
        if ((gamepad1previous.right_trigger > 0.3) && !(gamepad1current.right_trigger > 0.3)){
            return true;
        }
        return (gamepad1previous.left_trigger > 0.3) && !(gamepad1current.left_trigger > 0.3);
    }
    @Override
    public boolean linearActuatorUp(){
        return gamepad1current.dpad_right;
    }
    @Override
    public boolean linearActuatorDown(){
        return gamepad1current.dpad_left;
    }
    @Override
    public boolean specimenSampleIntake(){
        return gamepad2current.b && !gamepad2current.start;
    }
    @Override
    public boolean outtake(){
        return (gamepad2current.left_bumper && !gamepad2previous.left_bumper);
    }
    @Override
    public boolean intake(){
        return (gamepad2current.right_bumper && !gamepad2previous.right_bumper);
    }
    @Override
    public boolean grabSampleFromOutside(){
        return ((gamepad2current.right_trigger > 0.1) && !(gamepad2previous.right_trigger > 0.1)) || ((gamepad1current.x) && !(gamepad1previous.x));
    }
    @Override
    public boolean enterIntakePosition(){
        return ((gamepad2current.left_trigger > 0.1) && !(gamepad2previous.left_trigger > 0.1)) || ((gamepad1current.y) && !gamepad1previous.y);
    }
    @Override
    public boolean pickupAndDepositSpecimens(){
        return gamepad2current.y && !gamepad2previous.y;
    }
    @Override
    public boolean depositBack(){
        return gamepad2current.a && !gamepad2previous.a;
    }
    @Override
    public boolean escapePresets() {
        return gamepad2current.dpad_left;
    }
    @Override
    public boolean removeArmRules(){return false;}
    @Override
    public boolean diffUp(){return gamepad2current.dpad_down  || gamepad1current.b;}
    @Override
    public boolean diffDown(){return gamepad2current.dpad_up || gamepad1current.a;}
    @Override
    public boolean hang(){return gamepad2current.x;}
    @Override
    public boolean turnOffAutoGrab(){return gamepad2current.dpad_right;}
    @Override
    public boolean turnOffColorSensor(){return gamepad2current.touchpad;}
}
