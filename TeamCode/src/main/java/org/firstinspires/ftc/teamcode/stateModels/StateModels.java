package org.firstinspires.ftc.teamcode.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOpV5;
import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class StateModels {

    static DriveStates drivePresetState;
    static IntakeStates intakePresetState;
    static DepositStates depositPresetState;
    static Arm arm;
    static Wrist wrist;
    static DriverControls driverControls;
    static ElapsedTime timer;

    public static void initialize(Arm arm, Wrist wrist, DriverControls driverControls){
        StateModels.arm = arm;
        StateModels.wrist = wrist;
        StateModels.driverControls = driverControls;
    }

    public static void presetPositionDriveStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (drivePresetState){
            case START:
                if (driverControls.drivingPos()){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch, roll);
                    intakePresetState = IntakeStates.START;
                    depositPresetState = DepositStates.START;
                    drivePresetState = DriveStates.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    drivePresetState = DriveStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - slideLength < RobotConstants.SLIDE_TOLERANCE) {
                    arm.moveElbowToAngle(elbowAngle);
                    drivePresetState = DriveStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - elbowAngle) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    drivePresetState = DriveStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
        }
    }
    public static void presetPositionIntakeStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (intakePresetState){
            case START:
                if (driverControls.drivingPos()){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    drivePresetState = DriveStates.START;
                    depositPresetState = DepositStates.START;
                    intakePresetState = IntakeStates.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    intakePresetState = IntakeStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - slideLength < RobotConstants.SLIDE_TOLERANCE) {
                    arm.moveElbowToAngle(elbowAngle);
                    intakePresetState = IntakeStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - elbowAngle) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    intakePresetState = IntakeStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
        }
    }
    public static void presetPositionDepositStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (depositPresetState){
            case START:
                if (driverControls.depositReadyFrontTopBucket()){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    depositPresetState = DepositStates.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(0);
                    depositPresetState = DepositStates.RETRACTING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case RETRACTING_SLIDE:
                if (arm.getSlideExtension() - 0 < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - elbowAngle) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    depositPresetState = DepositStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - slideLength < RobotConstants.SLIDE_TOLERANCE) {
                    depositPresetState = DepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
        }
    }
}
