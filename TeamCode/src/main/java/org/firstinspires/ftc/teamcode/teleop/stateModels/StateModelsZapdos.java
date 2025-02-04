package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class StateModelsZapdos {

    public enum DepositCycles {START, GO_TO_SAFE_DRIVE, GO_TO_DEPOSIT, LEAVE_DEPOSIT}
    public enum SpecimenCycles {GO_TO_SPECIMEN_INTAKE, GO_TO_SPECIMEN_DEPOSIT}
    public enum IntakingSamplesForSpecimen {GO_TO_INTAKE, GO_TO_DROP_AND_BRING_IN}
    public enum BlockPickupType {NONE, INSIDE, OUTSIDE}
    static DriveStates drivePresetState;
    public static IntakeStates intakePresetState;
    static LeaveSubmersibleStates submersibleLeaveStates;
    public static EnterIntakePositionStates enterIntakePositionStates;
    static DepositStates depositPresetState;
    public static DepositStates depositBackPresetState;
    static DepositSampleIntoObservationZone depositSampleIntoObservationZone;
    static ExitDepositStates exitDepositPresetState;
   public static GrabBlockFromOutsideStates grabBlockFromOutsidePresetState;
    public static GrabBlockFromInsideStates grabBlockFromInsidePresetState;
    public static SpecimenPickupStates pickupSpecimenState;
    static SpecimenDepositStates depositSpecimenState;
    static HangStates hangState;
    static Arm arm;
    static Wrist wrist;
    static Claw claw;
    static ColorSensor color;
    static LinearActuator linearActuator;
    static DriverControls driverControls;
    static ElapsedTime timer;
    public static DepositCycles depositCycle;
    public static SpecimenCycles specimenCycle;
    public static IntakingSamplesForSpecimen specimenSampleIntake;
    public static BlockPickupType blockPickupType;
    public static boolean intakePosition;
    public static boolean endSpecimenDeposit;

    public static void initialize(Arm arm, Wrist wrist, Claw claw, LinearActuator linearActuator, DriverControls driverControls, ColorSensor color){
        StateModelsZapdos.arm = arm;
        StateModelsZapdos.wrist = wrist;
        StateModelsZapdos.driverControls = driverControls;
        StateModelsZapdos.claw = claw;
        StateModelsZapdos.linearActuator = linearActuator;
        StateModelsZapdos.color = color;

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
        depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
        enterIntakePositionStates = EnterIntakePositionStates.START;
        depositCycle = DepositCycles.START;
        specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
        specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
        hangState = HangStates.START;
        blockPickupType = BlockPickupType.NONE;
        intakePosition = false;
        endSpecimenDeposit = false;
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    drivePresetState = DriveStates.MOVING_WRIST;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
                    arm.moveSlideToLength(0);
                    drivePresetState = DriveStates.RETRACTING_SLIDE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case RETRACTING_SLIDE:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE) {
                    drivePresetState = DriveStates.WAITING_FOR_USER_INPUT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case WAITING_FOR_USER_INPUT:
                if (driverControls.drivingPos() || (driverControls.depositBack() && depositCycle == DepositCycles.GO_TO_SAFE_DRIVE)){
                    arm.moveElbowToAngle(elbowAngle);
                    drivePresetState = DriveStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    drivePresetState = DriveStates.START;
                }
                break;
            case MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE
                ){
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
                if ((driverControls.depositBack() && depositCycle == DepositCycles.START) || (driverControls.specimenSampleIntake() && specimenSampleIntake == IntakingSamplesForSpecimen.GO_TO_INTAKE)){ //intakePosition is true when the robot is ready to pick up a sample
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    intakePresetState = IntakeStates.MOVING_WRIST;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
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
                if (timer.milliseconds() > 250){
                    intakePosition = true;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_DROP_AND_BRING_IN;
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
                    hangState = HangStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    submersibleLeaveStates = LeaveSubmersibleStates.MOVING_WRIST;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    endSpecimenDeposit = false;
                }
                break;
            case MOVING_WRIST:
                if (timer.milliseconds() > 250){
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    depositPresetState = DepositStates.MOVING_WRIST;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
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
    public static void presetPositionDepositFrontStateModel(double pitch, double roll, double elbowAngle, double slideLength, double slideRetractionLength){
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    depositBackPresetState = DepositStates.RETRACTING_SLIDE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
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
    public static void depositSampleIntoBucketStateModel(double pitch, double roll, double elbowAngle, double intermediateElbowAngle, double slideLength){
        switch (exitDepositPresetState){
            case START:
                if (driverControls.depositBack() && depositCycle == DepositCycles.LEAVE_DEPOSIT){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    exitDepositPresetState = ExitDepositStates.OPENING_CLAW;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 200){
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
                    arm.moveElbowToAngle(intermediateElbowAngle);
                    arm.moveSlideToLength(slideLength);
                    exitDepositPresetState = ExitDepositStates.RETRACTING_SLIDES_AND_MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
            case RETRACTING_SLIDES_AND_MOVING_ELBOW:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE &&
                        (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE)){
                    intakePosition = true;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    exitDepositPresetState = ExitDepositStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    exitDepositPresetState = ExitDepositStates.START;
                }
                break;
        }
    }
    public static void presetPositionGrabBlockFromOutsideStateModel(double downPitch, double upPitch, double upRoll, double elbowIntakeDownAngle, double elbowIntakeUpAngle, double elbowAngle, double slideLengthBack){
        switch (grabBlockFromOutsidePresetState){
            case START:
                if (driverControls.grabSampleFromOutside()){
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    grabBlockFromOutsidePresetState = GrabBlockFromOutsideStates.ELBOW_DOWN;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    blockPickupType = BlockPickupType.OUTSIDE;
                    endSpecimenDeposit = false;
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
                if (timer.milliseconds()>300){
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
                if (timer.milliseconds() > 250) {
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_INTAKE_DOWN;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    blockPickupType = BlockPickupType.INSIDE;
                    endSpecimenDeposit = false;
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
                if (timer.milliseconds()>1500){
                    arm.moveElbowToAngle(elbowUpAngle);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.ELBOW_SLIGHTLY_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case ELBOW_SLIGHTLY_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE){
                    timer.reset();
                    wrist.presetPositionPitch(upPitch);
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.WRIST_MOVING_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    grabBlockFromInsidePresetState = GrabBlockFromInsideStates.START;
                }
                break;
            case WRIST_MOVING_UP:
                if (timer.milliseconds() > 250) {
                    wrist.presetPositionRoll(upRoll);
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
    public static void dropBlockAndMoveWristDown (double downPitch, double elbowAngle){
        switch (enterIntakePositionStates){
            case START:
                if (driverControls.enterIntakePosition() && intakePosition){
                    timer = new ElapsedTime();
                    timer.reset();
                    claw.openClaw();
                    arm.moveElbowToAngle(elbowAngle);
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    enterIntakePositionStates = EnterIntakePositionStates.INTERMEDIATE_CLAW;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    endSpecimenDeposit = false;
                }
                break;
            case INTERMEDIATE_CLAW:
                if (timer.milliseconds() > 200){
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
                if (timer.milliseconds() > 250){
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                }
                break;
        }
    }
    public static void depositSampleIntoObservationZone(double retractionLength, double pitchDown, double extensionLength,  double downPitch, double downRoll){
        switch (depositSampleIntoObservationZone){
            case START:
                if (driverControls.specimenSampleIntake() && specimenSampleIntake == IntakingSamplesForSpecimen.GO_TO_DROP_AND_BRING_IN){
                    timer = new ElapsedTime();
                    timer.reset();
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
                    hangState = HangStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    endSpecimenDeposit = false;
                    arm.moveSlideToLength(retractionLength);
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.RETRACT_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case RETRACT_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    wrist.presetPositionPitch(pitchDown);
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.WAIT_FOR_USER_INPUT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case WAIT_FOR_USER_INPUT:
                if (driverControls.specimenSampleIntake()) {
                    arm.moveSlideToLength(extensionLength);
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.EXTEND_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case EXTEND_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    claw.openClaw();
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.DEPOSIT_SAMPLE_INTO_OBSERVATION_ZONE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case DEPOSIT_SAMPLE_INTO_OBSERVATION_ZONE:
                if (timer.milliseconds() > 250){
                    wrist.presetPosition(downPitch,downRoll);
                    arm.moveSlideToLength(retractionLength);
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.RETRACT_SLIDES_AFTER_DEPOSIT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case RETRACT_SLIDES_AFTER_DEPOSIT:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
        }
    }
    public static void presetPositionPickupSpecimensStateModel(double pitch, double roll, double elbowAngle, double slideLength, double elbowUpAngle, double pickupSlideLength, double endSlideLength, double pitchEnd, double rollEnd){
        switch (pickupSpecimenState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && specimenCycle == SpecimenCycles.GO_TO_SPECIMEN_INTAKE){
                    timer = new ElapsedTime();
                    timer.reset();
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
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    /*if (endSpecimenDeposit){
                        claw.closeClaw();
                        pickupSpecimenState = SpecimenPickupStates.CLOSE_CLAW;
                    }else {
                        claw.openClaw();
                        pickupSpecimenState = SpecimenPickupStates.OPENING_CLAW;
                    }*/
                    claw.openClaw();
                    pickupSpecimenState = SpecimenPickupStates.OPENING_CLAW;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
                }
                break;
            case OPENING_CLAW:
                if (timer.milliseconds() > 200){
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
                double distance = color.getDistance(DistanceUnit.MM);
                if (driverControls.pickupAndDepositSpecimens() || (distance < 20)){
                    if (color.getDistance(DistanceUnit.MM) < 25){
                        arm.moveSlideToLength(pickupSlideLength);
                    }
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
                if (timer.milliseconds() > 200){
                    arm.moveElbowToAngle(elbowUpAngle);
                    pickupSpecimenState = SpecimenPickupStates.ELBOW_SLIGHTLY_UP;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                if (driverControls.enterIntakePosition()){
                    arm.holdArm();
                    claw.openClaw();
                }
                break;
            case ELBOW_SLIGHTLY_UP:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.LOW_ELBOW_TOLERANCE){
                    pickupSpecimenState = SpecimenPickupStates.WAITING_FOR_USER_INPUT_AGAIN;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case WAITING_FOR_USER_INPUT_AGAIN:
                if (driverControls.pickupAndDepositSpecimens()){
                    arm.moveSlideToLength(endSlideLength);
                    pickupSpecimenState = SpecimenPickupStates.MOVING_SLIDES;
                }
                if (driverControls.enterIntakePosition()){
                    arm.holdArm();
                    claw.openClaw();
                    arm.moveElbowToAngle(elbowAngle);
                    pickupSpecimenState = SpecimenPickupStates.MOVING_ELBOW;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case MOVING_SLIDES:
                if (arm.getSlideExtension() - arm.getSlideTargetPositionInInches() < RobotConstants.SLIDE_TOLERANCE){
                    timer.reset();
                    wrist.presetPosition(pitchEnd,rollEnd);
                    pickupSpecimenState = SpecimenPickupStates.MOVING_WRIST_TO_DEPOSIT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                break;
            case MOVING_WRIST_TO_DEPOSIT:
                if (timer.milliseconds() > 250){
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
    public static void presetPositionDepositSpecimensStateModel(double endPitch, double endRoll,double elbowDownAngle, double slideRetraction){
        switch (depositSpecimenState){
            case START:
                if (driverControls.pickupAndDepositSpecimens() && specimenCycle == SpecimenCycles.GO_TO_SPECIMEN_DEPOSIT){
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
                    pickupSpecimenState= SpecimenPickupStates.START;
                    enterIntakePositionStates = EnterIntakePositionStates.START;
                    hangState = HangStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    depositSpecimenState = SpecimenDepositStates.OPEN_CLAW;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
                }
                break;
            case OPEN_CLAW:
                if (timer.milliseconds() > 250){
                    timer.reset();
                    arm.moveSlideToLength(slideRetraction);
                    depositSpecimenState = SpecimenDepositStates.RETRACT_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case RETRACT_SLIDES:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    wrist.presetPosition(endPitch, endRoll);
                    depositSpecimenState = SpecimenDepositStates.MOVING_WRIST_TO_SAFE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
            case MOVING_WRIST_TO_SAFE:
                if (timer.milliseconds() > 250){
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
                    endSpecimenDeposit = true;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    depositSpecimenState = SpecimenDepositStates.START;
                    pickupSpecimenState = SpecimenPickupStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    depositSpecimenState = SpecimenDepositStates.START;
                }
                break;
        }
    }
    public static void hang(double pitch, double roll, double linearActuatorRetraction, double initialElbowAngle, double slideExtension, double hangElbowAngle, double elbowSlideCrossover, double slideRetration, double endElbowAngle){
        switch(hangState){
            case START:
                if (driverControls.hang()){
                    timer = new ElapsedTime();
                    timer.reset();
                    wrist.presetPosition(pitch, roll);
                    linearActuator.goToTargetPositionInches(linearActuatorRetraction);
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
                    depositSpecimenState = SpecimenDepositStates.START;
                    depositSampleIntoObservationZone = DepositSampleIntoObservationZone.START;
                    hangState = HangStates.LINEAR_ACTUATOR_DOWN;
                    depositCycle = DepositCycles.GO_TO_SAFE_DRIVE;
                    specimenCycle = SpecimenCycles.GO_TO_SPECIMEN_INTAKE;
                    specimenSampleIntake = IntakingSamplesForSpecimen.GO_TO_INTAKE;
                    intakePosition = false;
                    endSpecimenDeposit = false;
                }
                break;
            case LINEAR_ACTUATOR_DOWN:
                if ((Math.abs(linearActuator.getLinearActuatorPositionInches() - linearActuatorRetraction) < RobotConstants.LINEAR_ACTUATOR_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(initialElbowAngle);
                    hangState = HangStates.ELBOW_TO_SLIDE_EXTENSION_POSITION;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
                }
                break;
            case ELBOW_TO_SLIDE_EXTENSION_POSITION:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveSlideToLength(slideExtension);
                    hangState = HangStates.EXTENDING_SLIDES;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
                }
                break;
            case EXTENDING_SLIDES:
                if ((Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(hangElbowAngle);
                    hangState = HangStates.ELBOW_TO_HANG_POSITION;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
                }
                break;
            case ELBOW_TO_HANG_POSITION:
                if ((Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveSlideToLength(12.5);
                    hangState = HangStates.SLIDES_RETRACT;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
                }
                break;
            case SLIDES_RETRACT:
                if (((Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE))
                        && (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE)
                        && driverControls.hang()){
                    arm.moveElbowToAngle(endElbowAngle);
                    linearActuator.goToTargetPositionInches(9.5);
                    hangState = HangStates.ELBOW_TO_SAFE;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
                }
                break;
            case ELBOW_TO_SAFE:
                if (Math.abs(arm.getElbowAngleInDegrees() - arm.getElbowTargetPositionInDegrees()) < RobotConstants.ELBOW_TOLERANCE){
                    arm.moveSlideToLength(slideRetration);
                    hangState = HangStates.START;
                }
                if (driverControls.escapePresets()){
                    arm.holdArm();
                    hangState = HangStates.START;
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
