package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserDirective;

import java.util.HashSet;
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
    public boolean removeSpeedRules(){return gamepad1current.right_trigger > 0.5;}

    @Override
    public boolean slidesFullyUp() {
        return false;
        //return gamepad1.dpad_up;
    }

    @Override
    public boolean slidesFullyDown() {
        return false;
        //return gamepad1.dpad_down;
    }

    @Override
    public boolean pivotParallel() {
        //return gamepad2current.b
        return gamepad2current.dpad_right && !gamepad2current.dpad_left;
    }
    public boolean setNewPivotParallel(){
        return gamepad2current.dpad_right && gamepad2current.dpad_left;
    }

    @Override
    public boolean pivotPerp() {
        //return gamepad2.y
        return gamepad2current.dpad_up && !gamepad2current.dpad_left;
    }
    public boolean setNewPivotPerp(){
        return gamepad2current.dpad_up && gamepad2current.dpad_left;
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
            return -gamepad2current.left_stick_x;
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
        return false;
    }

    @Override
    public boolean intakeDirection() {
        return false;
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
        return gamepad2current.a && !gamepad2previous.a && !gamepad2previous.dpad_left;
    }

    @Override
    public boolean acsent1Park() {
        return gamepad2current.start && !gamepad2previous.start;
    }

    @Override
    public boolean drivingPos() {
        return gamepad2current.x && !gamepad2current.dpad_left;
    }

    @Override
    public boolean depositReadyBackTopBucket() {
        return gamepad2current.y && !gamepad2previous.y && !(gamepad2current.right_bumper) && !gamepad2current.dpad_left;
    }

    @Override
    public boolean depositReadyFrontTopBucket() {
        return gamepad2current.b && !gamepad2previous.b && !(gamepad2current.right_bumper) && !gamepad2current.dpad_left;
    }

    public boolean depositReadyBackBottomBucket(){
        return gamepad2current.y && !gamepad2previous.y && (gamepad2current.right_bumper) && !gamepad2current.dpad_left;
    }
    public boolean depositReadyFrontBottomBucket(){
        return gamepad2current.b && !gamepad2previous.b && (gamepad2current.right_bumper) && !gamepad2current.dpad_left;
    }
    public boolean setNewDrivingPos(){return gamepad2current.x && gamepad2current.dpad_left;}
    public boolean setNewDepositReadyBackTopBucket(){return gamepad2current.y && !gamepad2previous.y && !(gamepad2current.right_bumper) && gamepad2current.dpad_left;}
    public boolean setNewDepositReadyFrontTopBucket(){return gamepad2current.b && !gamepad2previous.b && !(gamepad2current.right_bumper) && gamepad2current.dpad_left;}
    public boolean setNewDepositReadyBackBottomBucket(){return gamepad2current.y && !gamepad2previous.y && (gamepad2current.right_bumper) && gamepad2current.dpad_left;}
    public boolean setNewDepositReadyFrontBottomBucket(){return gamepad2current.b && !gamepad2previous.b && (gamepad2current.right_bumper) && gamepad2current.dpad_left;}
    public boolean setNewSubmersibleIntakeReady(){return gamepad2current.a && !gamepad2previous.a && gamepad2current.dpad_left;}
    public boolean resetWrist() {
        return gamepad2current.back;
    }
    public boolean wristDown(){return gamepad2current.dpad_down;}
    public boolean isDriving(){return Math.abs(gamepad1current.left_stick_x) > 0 || Math.abs(gamepad1current.left_stick_y) > 0 || Math.abs(gamepad1current.right_stick_x) > 0;}
    public boolean removeArmRules(){return gamepad2current.left_bumper;}
    public boolean diffUp(){return -gamepad2current.right_stick_y > 0.5;}
    public boolean diffDown(){return -gamepad2current.right_stick_y < -0.5;}
    public boolean diffLeft(){return gamepad2current.right_stick_x > 0.5;}
    public boolean diffRight(){return gamepad2current.right_stick_x < -0.5;}
    public void rumbleArmGamepad(){gamepad2current.rumble(10);}
    public Set<UserDirective> getUserIntents(){
        Set<UserDirective> returnList = new HashSet<UserDirective>();
        if (isDriving()){
            if (microDriveAdjustments()){
                returnList.add(UserDirective.MANUAL_DRIVE_ADJUSTMENTS);
            } else {
                returnList.add(UserDirective.MANUAL_DRIVE_NORMAL);
            }
        }
        if (driveTypeSwitch()){
            returnList.add(UserDirective.DRIVE_SWITCH);
        }
        if (slowMode()){
            returnList.add(UserDirective.SPEED_SWITCH);
        }
        if(resetIMU()){
            returnList.add(UserDirective.IMU_RESET);
        }
        if(Math.abs(pivotJoystick()) > 0){
            returnList.add(UserDirective.MANUAL_ELBOW);
        }
        if(Math.abs(slideMovement()) > 0){
            returnList.add(UserDirective.MANUAL_SLIDE);
        }
        if(depositReadyBackTopBucket()){
            returnList.add(UserDirective.PRESET_DEPOSIT_BACK_TOP);
        }
        if(depositReadyFrontTopBucket()){
            returnList.add(UserDirective.PRESET_DEPOSIT_FRONT_TOP);
        }
        if(depositReadyBackBottomBucket()){
            returnList.add(UserDirective.PRESET_DEPOSIT_BACK_BOTTOM);
        }
        if(depositReadyFrontBottomBucket()){
            returnList.add(UserDirective.PRESET_DEPOSIT_FRONT_BOTTOM);
        }
        if(drivingPos()){
            returnList.add(UserDirective.PRESET_SAFE_DRIVING_POSITION);
        }
        if(submersibleIntakeReady()){
            returnList.add(UserDirective.PRESET_SUBMERSIBLE_INTAKE);
        }
        if(intakenewBackward() > 0.5){
            returnList.add(UserDirective.INTAKE_BACKWARD);
        }
        if(intakenewForward() > 0.5){
            returnList.add(UserDirective.INTAKE_FORWARD);
        }
        if (pivotParallel()){
            returnList.add(UserDirective.ELBOW_0);
        }
        if (pivotPerp()){
            returnList.add(UserDirective.ELBOW_90);
        }
        if (removeSpeedRules()){
            returnList.add(UserDirective.REMOVE_SPEED_RULES);
        }
        if (wristDown()){
            returnList.add(UserDirective.WRIST_DOWN);
        }
        if (setNewDepositReadyBackBottomBucket()){
            returnList.add(UserDirective.SET_DEPOSIT_BACK_BOTTOM);
        }
        if (setNewDepositReadyBackTopBucket()){
            returnList.add(UserDirective.SET_DEPOSIT_BACK_TOP);
        }
        if (setNewDepositReadyFrontBottomBucket()){
            returnList.add(UserDirective.SET_DEPOSIT_FRONT_BOTTOM);
        }
        if (setNewDepositReadyFrontTopBucket()){
            returnList.add(UserDirective.SET_DEPOSIT_FRONT_TOP);
        }
        if (setNewDrivingPos()){
            returnList.add(UserDirective.SET_SAFE_DRIVING_POSITION);
        }
        if (setNewSubmersibleIntakeReady()){
            returnList.add(UserDirective.SET_SUBMERSIBLE_INTAKE);
        }
        if (setNewPivotParallel()){
            returnList.add(UserDirective.SET_ELBOW_0);
        }
        if (setNewPivotPerp()){
            returnList.add(UserDirective.SET_ELBOW_90);
        }
        if (removeArmRules()){
            returnList.add(UserDirective.REMOVE_ARM_RULES);
        }
        if (diffUp()){
            returnList.add(UserDirective.DIFF_UP);
        }
        if (diffDown()){
            returnList.add(UserDirective.DIFF_DOWN);
        }
        if (diffRight()){
            returnList.add(UserDirective.DIFF_RIGHT);
        }
        if (diffLeft()){
            returnList.add(UserDirective.DIFF_LEFT);
        }
        return returnList;
    }
}
