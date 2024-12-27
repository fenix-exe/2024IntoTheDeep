package org.firstinspires.ftc.teamcode.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class StateModels {

    static DriveStates drivePresetState;
    static IntakeStates intakePresetState;
    static DepositStates depositPresetState;
    public static DepositStates depositBackPresetState;
    static ExitDepositStates exitDepositPresetState;
    static GrabBlockFromOutsideStates grabBlockFromOutsidePresetState;
    static GrabBlockFromInsideStates grabBlockFromInsidePresetState;
    static SpecimenPickupStates pickupSpecimenState;
    static SpecimenDepositStates depositSpecimenState;
    static Arm arm;
    static Wrist wrist;
    static Claw claw;
    static DriverControls driverControls;
    static ElapsedTime timer;

    public static void initialize(Arm arm, Wrist wrist, Claw claw, DriverControls driverControls){
        StateModels.arm = arm;
        StateModels.wrist = wrist;
        StateModels.driverControls = driverControls;
        StateModels.claw = claw;

        drivePresetState = DriveStates.START;
        intakePresetState = IntakeStates.START;
        depositPresetState = DepositStates.START;
        exitDepositPresetState = ExitDepositStates.START;
        grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
        grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
        pickupSpecimenState= SpecimenPickupStates.START;
        depositSpecimenState = SpecimenDepositStates.START;
        depositBackPresetState = DepositStates.START;
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
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
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
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    arm.moveElbowToAngle(elbowAngle);
                    drivePresetState = DriveStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
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
                if (driverControls.submersibleIntakeReady()){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    wrist.presetPosition(pitch,roll);
                    drivePresetState = DriveStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    intakePresetState = IntakeStates.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(0);
                    intakePresetState = IntakeStates.RETRACTING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case RETRACTING_SLIDE:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    intakePresetState = IntakeStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    intakePresetState = IntakeStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
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
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    depositBackPresetState = DepositStates.START;
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
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    depositPresetState = DepositStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    depositPresetState = DepositStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    depositPresetState = DepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
        }
    }
    public static void presetPositionDepositBackStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (depositBackPresetState){
            case START:
                if (driverControls.depositBack()){
                    timer = new ElapsedTime();
                    timer.reset();
                    arm.moveSlideToLength(0);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.RETRACTING_SLIDE;
                }
                break;
            case RETRACTING_SLIDE:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    depositBackPresetState = DepositStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    depositBackPresetState = DepositStates.MOVING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
            case MOVING_SLIDE:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    depositBackPresetState = DepositStates.MOVING_WRIST;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    depositBackPresetState = DepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
        }
    }
    public static void depositSampleIntoBucketStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (exitDepositPresetState){
            case START:
                if (driverControls.leaveDeposit()){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    exitDepositPresetState = ExitDepositStates.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 250){
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    exitDepositPresetState = ExitDepositStates.MOVING_WRIST;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    exitDepositPresetState = ExitDepositStates.RETRACTING_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
            case RETRACTING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    exitDepositPresetState = ExitDepositStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    exitDepositPresetState = ExitDepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
        }
    }
    public static void presetPositionGrabBlockFromOutsideStateModel(double downPitch, double upPitch, double upRoll, double elbowIntakeDownAngle, double elbowIntakeUpAngle, double elbowAngle, double slideLength){
        switch (grabBlockFromOutsidePresetState){
            case START:
                if (driverControls.grabSampleFromOutside()){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    arm.moveElbowToAngle(elbowIntakeDownAngle);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.ELBOW_DOWN;
                }
                break;
            case ELBOW_DOWN:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE ){
                    timer.reset();
                    claw.openClaw();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.INTAKE_OPENING;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case INTAKE_OPENING:
                if (timer.milliseconds() > 500){
                    timer.reset();
                    wrist.presetPositionPitch(downPitch);
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.WRIST_MOVING_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case WRIST_MOVING_DOWN:
                if (timer.milliseconds() > 500){
                    timer.reset();
                    claw.closeClaw();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.INTAKE_CLOSING;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case INTAKE_CLOSING:
                if (timer.milliseconds()>500){
                    arm.moveElbowToAngle(elbowIntakeUpAngle);
                    //wrist.presetPosition(upPitch, upRoll);
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.ELBOW_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case ELBOW_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE ){
                    timer.reset();
                    wrist.presetPosition(upPitch, upRoll);
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.WRIST_MOVING_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case WRIST_MOVING_UP:
                if (timer.milliseconds() > 500) {
                    //arm.moveSlideToLength(slideLength);
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            /*case SLIDES_RETRACTING:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.ELBOW_MOVING_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;
            case ELBOW_MOVING_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                }
                break;*/
        }
    }
    public static void presetPositionGrabBlockFromInsideStateModel(double downPitch, double upPitch, double upRoll, double elbowDownAngle, double elbowUpAngle, double elbowAngle, double slideLength){
        switch (grabBlockFromInsidePresetState){
            case START:
                if (driverControls.grabSampleFromInside()){
                    timer = new ElapsedTime();
                    arm.moveElbowToAngle(elbowUpAngle);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_INTAKE_UP;
                }
                break;
            case ELBOW_INTAKE_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    timer.reset();
                    claw.intermediateClaw();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.INTAKE_MOVING_TO_INTERMEDIATE_POSITION;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case INTAKE_MOVING_TO_INTERMEDIATE_POSITION:
                if (timer.milliseconds() > 100){
                    timer.reset();
                    wrist.presetPositionPitch(downPitch);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.WRIST_MOVING_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case WRIST_MOVING_DOWN:
                if (timer.milliseconds() > 250){
                    arm.moveElbowToAngle(elbowDownAngle);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_INTAKE_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case ELBOW_INTAKE_DOWN:{
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowAngleInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    timer.reset();
                    claw.openClaw();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.INTAKE_GRABBING_BLOCK;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            }
            case INTAKE_GRABBING_BLOCK:
                if (timer.milliseconds()>100){
                    arm.moveElbowToAngle(elbowUpAngle);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_SLIGHTLY_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case ELBOW_SLIGHTLY_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    timer.reset();
                    wrist.presetPosition(upPitch, upRoll);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.WRIST_MOVING_UP;
                }
            case WRIST_MOVING_UP:
                if (timer.milliseconds() > 250) {
                    //arm.moveSlideToLength(slideLength);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            /*case SLIDES_RETRACTING:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_MOVING_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case ELBOW_MOVING_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;*/
        }
    }
    public static void presetPositionPickupSpecimensStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (pickupSpecimenState){
            case START:
                if (driverControls.pickupSpecimenPreset()){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.OPENING_CLAW;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 100){
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    pickupSpecimenState = SpecimenPickupStates.MOVING_WRIST;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(0);
                    pickupSpecimenState = SpecimenPickupStates.RETRACTING_SLIDES;;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case RETRACTING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    pickupSpecimenState = SpecimenPickupStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideLength);
                    pickupSpecimenState = SpecimenPickupStates.EXTENDING_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case EXTENDING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;

        }
    }
    public static void presetPositionDepositSpecimensStateModel(double pitch, double roll, double elbowAngle, double slideDepositLength){
        switch (depositSpecimenState){
            case START:
                if (driverControls.depositSpecimenPreset()){
                    timer = new ElapsedTime();
                    timer.reset();
                    arm.moveSlideToLength(0);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.RETRACTING_SLIDES;
                }
                break;
            case RETRACTING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    depositSpecimenState = SpecimenDepositStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideDepositLength);
                    depositSpecimenState = SpecimenDepositStates.EXTENDING_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case EXTENDING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    depositSpecimenState = SpecimenDepositStates.MOVING_WRIST;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    depositSpecimenState = SpecimenDepositStates.START;;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            /*case OPENING_CLAW:
                if (timer.milliseconds() > 100){
                    arm.moveSlideToLength(0);
                    depositSpecimenState = SpecimenDepositStates.RETRACTING_SLIDES_AT_END;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case RETRACTING_SLIDES_AT_END:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;*/

        }
    }


}
