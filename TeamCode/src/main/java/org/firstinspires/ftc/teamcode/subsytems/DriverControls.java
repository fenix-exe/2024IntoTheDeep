package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserIntent;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

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
    public boolean microDriveAdjustments(){
        return gamepad1current.left_trigger > 0.5;
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
        if (Math.abs(gamepad2current.left_stick_y) > 0.65){
            return -gamepad2current.left_stick_y;
        }
        return 0;
    }

    @Override
    public double slideMovement() {
        if (Math.abs(gamepad2current.left_stick_x) > 0.5){
            return gamepad2current.left_stick_x;
        }
        return 0;
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
        return gamepad2current.a && !gamepad2previous.a;
    }

    @Override
    public boolean acsent1Park() {
        return gamepad2current.start && !gamepad2previous.start;
    }

    @Override
    public boolean drivingPos() {
        return gamepad2current.x;
    }

    @Override
    public boolean depositReadyBackTopBucket() {
        return gamepad2current.y && !gamepad2previous.y && !(gamepad2current.right_trigger > 0.5);
    }

    @Override
    public boolean depositReadyFrontTopBucket() {
        return gamepad2current.b && !gamepad2previous.b && !(gamepad2current.right_trigger > 0.5);
    }

    public boolean depositReadyBackBottomBucket(){
        return gamepad2current.y && !gamepad2previous.y && (gamepad2current.right_trigger > 0.5);
    }
    public boolean depositReadyFrontBottomBucket(){
        return gamepad2current.b && !gamepad2previous.b && (gamepad2current.right_trigger > 0.5);
    }

    public boolean resetWrist() {
        return gamepad2current.back;
    }
    public boolean isDriving(){return Math.abs(gamepad1current.left_stick_x) > 0 || Math.abs(gamepad1current.left_stick_y) > 0 || Math.abs(gamepad1current.right_stick_x) > 0;}
    public Set<UserIntent> getUserIntents(){
        Set<UserIntent> returnList = new HashSet<UserIntent>();
        if (isDriving()){
            if (microDriveAdjustments()){
                returnList.add(UserIntent.MANUAL_DRIVE_ADJUSTMENTS);
            } else {
                returnList.add(UserIntent.MANUAL_DRIVE_NORMAL);
            }
        }
        if (driveTypeSwitch()){
            returnList.add(UserIntent.DRIVE_SWITCH);
        }
        if (slowMode()){
            returnList.add(UserIntent.SPEED_SWITCH);
        }
        if(resetIMU()){
            returnList.add(UserIntent.IMU_RESET);
        }
        if(Math.abs(pivotJoystick()) > 0){
            returnList.add(UserIntent.MANUAL_ELBOW);
        }
        if(Math.abs(slideMovement()) > 0){
            returnList.add(UserIntent.MANUAL_SLIDE);
        }
        if(depositReadyBackTopBucket()){
            returnList.add(UserIntent.PRESET_DEPOSIT_BACK_TOP);
        }
        if(depositReadyFrontTopBucket()){
            returnList.add(UserIntent.PRESET_DEPOSIT_FRONT_TOP);
        }
        if(depositReadyBackBottomBucket()){
            returnList.add(UserIntent.PRESET_DEPOSIT_BACK_BOTTOM);
        }
        if(depositReadyFrontBottomBucket()){
            returnList.add(UserIntent.PRESET_DEPOSIT_FRONT_BOTTOM);
        }
        if(drivingPos()){
            returnList.add(UserIntent.PRESET_SAFE_DRIVING_POSITION);
        }
        if(submersibleIntakeReady()){
            returnList.add(UserIntent.PRESET_SUBMERSIBLE_INTAKE);
        }
        return returnList;
    }
}
