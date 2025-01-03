package org.firstinspires.ftc.teamcode.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

public class StateModels {

    public enum DepositCycles {GO_TO_SAFE_DRIVE, GO_TO_DEPOSIT, LEAVE_DEPOSIT}
    public enum SpecimenCycles {GO_TO_SPECIMEN_INTAKE, GO_TO_SPECIMEN_DEPOSIT}
    public enum BlockPickupType {NONE, INSIDE, OUTSIDE}
    static DriveStates drivePresetState;
    public static IntakeStates intakePresetState;
    static LeaveSubmersibleStates submersibleLeaveStates;
    public static EnterIntakePositionStates enterIntakePositionStates;
    static DepositStates depositPresetState;
    public static DepositStates depositBackPresetState;
    static ExitDepositStates exitDepositPresetState;
    static GrabBlockFromOutsideStates grabBlockFromOutsidePresetState;
    public static GrabBlockFromInsideStates grabBlockFromInsidePresetState;
    static SpecimenPickupStates pickupSpecimenState;
    static SpecimenDepositStates depositSpecimenState;
    static Arm arm;
    static Wrist wrist;
    static Claw claw;
    static DriverControls driverControls;
    static ElapsedTime timer;
    public static DepositCycles depositCycle;
    public static SpecimenCycles specimenCycle;
    public static BlockPickupType blockPickupType;
    public static boolean intakePosition;

    public static void initialize(Arm arm, Wrist wrist, Claw claw, DriverControls driverControls){
        StateModels.arm = arm;
        StateModels.wrist = wrist;
        StateModels.driverControls = driverControls;
        StateModels.claw = claw;

        drivePresetState = DriveStates.START;
        intakePresetState = IntakeStates.START;
        submersibleLeaveStates = LeaveSubmersibleStates.START;
        depositPresetState = DepositStates.START;
        exitDepositPresetState = ExitDepositStates.START;
        grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
        grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
        pickupSpecimenState= SpecimenPickupStates.START;
        depositSpecimenState = SpecimenDepositStates.START;
        depositBackPresetState = DepositStates.START;
        enterIntakePositionStates = EnterIntakePositionStates.START;
        depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
        specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
        blockPickupType = BlockPickupType.NONE;
        intakePosition = false;
    }

    public static void presetPositionDriveStateModel(double pitch, double elbowAngle, double slideLength){
        switch (drivePresetState){
            case START:
                if (driverControls.drivingPos() || (driverControls.depositBack() && depositCycle == DepositCycles.GO_TO_SAFE_DRIVE)){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPositionPitch(pitch);
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    drivePresetState = DriveStates.MOVING_WRIST;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(slideLength);
                    drivePresetState = DriveStates.RETRACTING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case RETRACTING_SLIDE:
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
                    drivePresetState = DriveStates.EXTENDING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case EXTENDING_SLIDE:
                if (Math.abs(arm.getSlideExtension() -  arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    depositCycle = DepositCycles.GO_TO_DEPOSIT;
                    drivePresetState = DriveStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
        }
    }
    public static void presetPositionIntakeStateModel(double pitch, double roll, double downPitch, double downRoll, double elbowAngle, double slideLength){
        switch (intakePresetState){
            case START:
                if (driverControls.submersibleIntakeReady() && !intakePosition){ //intakePosition is true when the robot is ready to pick up a sample
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.intermediateClaw();
                    wrist.presetPosition(pitch,roll);
                    drivePresetState = DriveStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    intakePresetState = IntakeStates.MOVING_WRIST;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
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
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    timer.reset();
                    wrist.presetPosition(downPitch, downRoll);
                    intakePresetState = IntakeStates.MOVING_WRIST_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
            case MOVING_WRIST_DOWN:
                if (timer.milliseconds() > 500){
                    intakePosition = true;
                    intakePresetState = IntakeStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    intakePresetState = IntakeStates.START;
                }
                break;
        }
    }
    public static void leaveSubmersibleStateModel(double pitch, double roll, double elbowAngle){
        switch (submersibleLeaveStates) {
            case START:
                if (driverControls.submersibleIntakeReady() && intakePosition) {
                    timer = new ElapsedTime();
                    timer.reset();
                    if (blockPickupType == BlockPickupType.INSIDE){
                        wrist.presetPosition(pitch, roll);
                    } else {
                        wrist.presetPosition(pitch, roll+90);
                    }
                    drivePresetState = DriveStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState = SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    intakePresetState = IntakeStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.MOVING_WRIST;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 500){
                    arm.moveSlideToLength(0);
                    submersibleLeaveStates = LeaveSubmersibleStates.RETRACTING_SLIDES;
                }
                if (driverControls.escapePresets()) {
                    arm.holdArm();
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                }
                break;
            case RETRACTING_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE) {
                    intakePosition = false;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                }
                if (driverControls.escapePresets()) {
                    arm.holdArm();
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                }
                break;
        }
    }
    public static void presetPositionDepositStateModel(double pitch, double roll, double elbowAngle, double slideLength){
        switch (depositPresetState){
            case START:
                if (driverControls.depositReadyFrontTopBucket() && depositCycle == DepositCycles.GO_TO_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch, roll);
                    /*if (blockPickupType == BlockPickupType.INSIDE){
                        wrist.presetPosition(pitch, roll);
                    } else {
                        wrist.presetPosition(pitch, roll+90);
                    }*/
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    depositPresetState = DepositStates.MOVING_WRIST;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
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
                    arm.moveSlideToLength(slideLength);
                    depositPresetState = DepositStates.MOVING_ELBOW_AND_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
            case MOVING_ELBOW_AND_SLIDE:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)){
                    depositPresetState = DepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositPresetState = DepositStates.START;
                }
                break;
        }
    }
    public static void presetPositionDepositBackStateModel(double pitch, double roll, double elbowAngle, double slideLength, double slideRetractionLength){
        switch (depositBackPresetState){
            case START:
                if (driverControls.depositBack() && depositCycle == DepositCycles.GO_TO_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPositionPitch(0);
                    arm.moveSlideToLength(slideRetractionLength);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    depositPresetState = DepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    depositBackPresetState = DepositStates.RETRACTING_SLIDE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
                }
                break;
            case RETRACTING_SLIDE:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowAngle);
                    arm.moveSlideToLength(slideLength);
                    depositBackPresetState = DepositStates.MOVING_ELBOW_AND_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
            case MOVING_ELBOW_AND_SLIDE:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)){
                    timer.reset();
                    if (blockPickupType == BlockPickupType.INSIDE){
                        wrist.presetPosition(pitch, roll-90);
                    } else {
                        wrist.presetPosition(pitch, roll);
                    }
                    depositBackPresetState = DepositStates.MOVING_WRIST;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositBackPresetState = DepositStates.START;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    if (blockPickupType == BlockPickupType.OUTSIDE){
                        claw.openClaw();
                    } else {
                        claw.closeClaw();
                    }
                    blockPickupType = BlockPickupType.NONE;
                    depositCycle = DepositCycles.LEAVE_DEPOSIT;
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
                if (driverControls.depositBack() && depositCycle == DepositCycles.LEAVE_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch,roll);
                    /*if (blockPickupType == BlockPickupType.OUTSIDE){
                        claw.openClaw();
                    } else {
                        claw.closeClaw();
                    }*/
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    exitDepositPresetState = ExitDepositStates.MOVING_WRIST;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 250){
                    blockPickupType = BlockPickupType.NONE;
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
                    depositCycle = DepositCycles.GO_TO_DEPOSIT;
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
                if (driverControls.grabSampleFromOutside() && depositCycle != DepositCycles.LEAVE_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    arm.moveElbowToAngle(elbowIntakeDownAngle);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.ELBOW_DOWN;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    blockPickupType = BlockPickupType.OUTSIDE;
                }
                break;
            case ELBOW_DOWN:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE ){
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
                if (timer.milliseconds()>200){
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
                if (driverControls.grabSampleFromInside() && depositCycle != DepositCycles.LEAVE_DEPOSIT){
                    timer = new ElapsedTime();
                    arm.moveElbowToAngle(elbowDownAngle);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_INTAKE_DOWN;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    blockPickupType = BlockPickupType.INSIDE;
                }
                break;
            case ELBOW_INTAKE_DOWN:{
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE){
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
                if (timer.milliseconds()>200){
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
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case WRIST_MOVING_UP:
                if (timer.milliseconds() > 500) {
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
    public static void dropBlockAndMoveWristDown (double downPitch){
        switch (enterIntakePositionStates){
            case START:
                if (driverControls.enterIntakePosition() && intakePosition){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.intermediateClaw();
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.INTERMEDIATE_CLAW;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                }
                break;
            case INTERMEDIATE_CLAW:
                if (timer.milliseconds() > 500){
                    timer.reset();
                    wrist.presetPosition(downPitch, 0);
                    enterIntakePositionStates = EnterIntakePositionStates.WRIST_MOVING_DOWN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                }
                break;
            case WRIST_MOVING_DOWN:
                if (timer.milliseconds() > 500){
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                }
                break;
        }
    }
    public static void presetPositionPickupSpecimensStateModel(double pitch, double roll, double elbowAngle, double slideLength, double elbowUpAngle){
        switch (pickupSpecimenState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && specimenCycle == SpecimenCycles.GO_TO_SPECIMEN_INTAKE){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    depositSpecimenState = SpecimenDepositStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    pickupSpecimenState= SpecimenPickupStates.OPENING_CLAW;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
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
                    pickupSpecimenState = SpecimenPickupStates.RETRACTING_SLIDES;
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
                    pickupSpecimenState = SpecimenPickupStates.WAITING_FOR_USER_INPUT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case WAITING_FOR_USER_INPUT:
                if (driverControls.pickupAndDepositSpecimens()){
                    timer.reset();
                    claw.closeClaw();
                    pickupSpecimenState = SpecimenPickupStates.CLOSE_CLAW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case CLOSE_CLAW:
                if (timer.milliseconds() > 500){
                    pickupSpecimenState = SpecimenPickupStates.WAITING_FOR_USER_INPUT_AGAIN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                if (driverControls.enterIntakePosition()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.WAITING_FOR_USER_INPUT;
                }
                break;
            case WAITING_FOR_USER_INPUT_AGAIN:
                if (driverControls.pickupAndDepositSpecimens()){
                    arm.moveElbowToAngle(elbowUpAngle);
                    pickupSpecimenState = SpecimenPickupStates.ELBOW_SLIGHTLY_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case ELBOW_SLIGHTLY_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE){
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_DEPOSIT;
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
        }
    }
    public static void presetPositionDepositSpecimensStateModel(double pitch, double roll, double elbowAngle, double elbowDownAngle, double slideStartLength, double slideDepositLength){
        switch (depositSpecimenState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && specimenCycle == SpecimenCycles.GO_TO_SPECIMEN_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    arm.moveSlideToLength(slideStartLength);
                    drivePresetState = DriveStates.START;
                    intakePresetState = IntakeStates.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.START;
                    exitDepositPresetState = ExitDepositStates.START;
                    depositPresetState = DepositStates.START;
                    depositBackPresetState = DepositStates.START;
                    pickupSpecimenState= SpecimenPickupStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    depositSpecimenState = SpecimenDepositStates.MOVING_SLIDES;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                }
                break;
            case MOVING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
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
                    arm.moveSlideToLength(slideStartLength);
                    depositSpecimenState = SpecimenDepositStates.WAITING_FOR_USER_INPUT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case WAITING_FOR_USER_INPUT:
                if (driverControls.pickupAndDepositSpecimens()){
                    arm.moveSlideToLength(slideDepositLength);
                    depositSpecimenState = SpecimenDepositStates.SLIGHTLY_EXTEND_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case SLIGHTLY_EXTEND_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.LOW_SLIDE_TOLERANCE){
                    depositSpecimenState = SpecimenDepositStates.WAITING_AGAIN_FOR_USER_INPUT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case WAITING_AGAIN_FOR_USER_INPUT:
                if (driverControls.pickupAndDepositSpecimens()){
                    timer.reset();
                    claw.openClaw();
                    depositSpecimenState = SpecimenDepositStates.OPEN_CLAW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case OPEN_CLAW:
                if (timer.milliseconds() > 500){
                    timer.reset();
                    wrist.presetPosition(-pitch, roll);
                    depositSpecimenState = SpecimenDepositStates.MOVING_WRIST_TO_SAFE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case MOVING_WRIST_TO_SAFE:
                if (timer.milliseconds() > 500){
                    arm.moveSlideToLength(0);
                    depositSpecimenState = SpecimenDepositStates.RETRACT_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case RETRACT_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    arm.moveElbowToAngle(elbowDownAngle);
                    depositSpecimenState = SpecimenDepositStates.MOVING_ELBOW_TO_SAFE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case MOVING_ELBOW_TO_SAFE:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
        }
    }
    public static String getDebugString(){
        return drivePresetState + ", " + intakePresetState + ", " + submersibleLeaveStates + ", "
                + depositBackPresetState + ", " + exitDepositPresetState + ", " + grabBlockFromOutsidePresetState + ", "
                + grabBlockFromInsidePresetState + ", " + enterIntakePositionStates + ", " + pickupSpecimenState + ", "
                + depositSpecimenState;
    }


}
