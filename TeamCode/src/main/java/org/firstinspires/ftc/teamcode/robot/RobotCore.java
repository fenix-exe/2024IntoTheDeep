package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmActionList;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsytems.arm.HoldElbowAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.HoldSlideAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveElbowAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveElbowToPresetAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveSlideAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveToPresetPositionAction;
import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserIntent;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.ChangeSpeedAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainActionList;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainSpeedMultiplier;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainState;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.MoveDriveTrainAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.ResetIMUAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.StopDriveTrainAction;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffector;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorActionList;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorState;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.MoveEndEffectorToPresetPositionAction;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RobotCore {
    static Arm arm;
    static DriveTrain drivetrain;

    static DriverControls driverControls;
    static EndEffector endEffector;

    private static Set<RobotActivityState> activityState;
    public static void initialize(DriverControls driverControls, DriveTrain drivetrain, Arm arm, EndEffector endEffector){
        activityState =new HashSet<>();
        RobotCore.arm = arm;
        RobotCore.drivetrain = drivetrain;
        RobotCore.driverControls = driverControls;
        RobotCore.endEffector = endEffector;
    }
    public static Set<RobotActivityState> getCurrentActivityState(){
        return activityState;
    }
    /*public static RobotPhysicalState getRobotPhysicalState(){
        RobotPhysicalState physicalState = new RobotPhysicalState();
        physicalState.elbowAngleInDegrees = arm.getElbowAngleInDegrees();
        physicalState.slidePositionInInches = arm.getSlideLengthInInches();
        return physicalState;
    }*/
    /*public static DriveTrainActionList getStepsForUserIntentForDriveTrain(Set<UserIntent> intent, RobotPhysicalState currentPhysicalState, Set<RobotActivityState> currentActivityState){
        DriveTrainActionList actions = new DriveTrainActionList();
        if(intent.contains(UserIntent.SPEED_SWITCH)){
            actions.add(new ChangeSpeedAction(drivetrain));
        }
        if (intent.contains(UserIntent.IMU_RESET)) {
            actions.add(new ResetIMUAction(drivetrain));
        }
        if (intent.contains(UserIntent.MANUAL_DRIVE)) {
            double speedMultiplier = currentPhysicalState.getSpeedMultiplier();
            if (speedMultiplier != DriveTrainSpeedMultiplier.NO_MULTIPLIER){
                actions.add(new MoveDriveTrainAction(drivetrain, speedMultiplier));
            } else {
                actions.add(new MoveDriveTrainAction(drivetrain));
            }
        }
        return actions;
    }

    public static ArmActionList getStepsForUserIntentForArm(Set<UserIntent> intent, RobotPhysicalState currentPhysicalState, Set<RobotActivityState> currentActivityState){
        ArmActionList actions = new ArmActionList();
        if(intent.contains(UserIntent.PRESET_DEPOSIT_BACK_TOP) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_DEPOSIT_BACK_TOP)){
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_BOTTOM) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_DEPOSIT_BACK_BOTTOM)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_TOP) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_DEPOSIT_FRONT_TOP)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        }else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_BOTTOM) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_DEPOSIT_FRONT_BOTTOM)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_BOTTOM) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_DEPOSIT_BACK_BOTTOM)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        }else if (intent.contains(UserIntent.PRESET_SUBMERSIBLE_INTAKE) || currentActivityState.contains(RobotActivityState.TRANSITION_TO_SUBMERSIBLE_INTAKE)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.INTAKE_POSITION));
        }

        if (intent.contains(UserIntent.MANUAL_DRIVE)) {
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.SAFE_DRIVING_POSITION));
        }

        // this is tricky - we have to cancel preset and give full control to driver
        if(intent.contains(UserIntent.MANUAL_ELBOW) || intent.contains(UserIntent.MANUAL_SLIDE)) {
            actions.clear();
            if(intent.contains(UserIntent.MANUAL_ELBOW)) {
                actions.add(new MoveElbowByJoystickAction(arm));
            }
            if(intent.contains(UserIntent.MANUAL_SLIDE)) {
                actions.add(new MoveSlideAction(arm,));
            }
        }
        return actions;
    }

    public static EndEffectorActionList getStepsForUserIntentForEndEffector(Set<UserIntent> intent, RobotPhysicalState currentPhysicalState, Set<RobotActivityState> currentActivityState){
        EndEffectorActionList actions = new EndEffectorActionList();
        return actions;
    }

    public static void setCurrentActivityState(DriveTrainState driveActivityState, ArmState armActivityState, EndEffectorState endEffectorActivityState, Set<UserIntent> intent){
        if ((driveActivityState == DriveTrainState.MOVING)){
            activityState.add(RobotActivityState.MANUAL_DRIVE);
        } else {
            activityState.remove(RobotActivityState.MANUAL_DRIVE);
        }

        if (armActivityState == ArmState.MOVING_TO_PRESET && arm.isBusy() && arm.hasTargetPreset()) {
            // We are still moving to the preset location so dont remove the activity that was added on preset button press
        } else {
            activityState.remove(RobotActivityState.TRANSITION_TO_DEPOSIT_BACK_TOP);
            activityState.remove(RobotActivityState.TRANSITION_TO_DEPOSIT_BACK_BOTTOM);
            activityState.remove(RobotActivityState.TRANSITION_TO_DEPOSIT_FRONT_TOP);
            activityState.remove(RobotActivityState.TRANSITION_TO_DEPOSIT_FRONT_BOTTOM);
            activityState.remove(RobotActivityState.TRANSITION_TO_SUBMERSIBLE_INTAKE);
            activityState.remove(RobotActivityState.TRANSITION_TO_SAFE_DRIVING_POSITION);
        }

        if (armActivityState == ArmState.MANUAL_MOVE && arm.isBusy()) {
            activityState.add(RobotActivityState.MANUAL_ARM);
        } else {
            activityState.remove(RobotActivityState.MANUAL_ARM);
        }

    }*/

    public static void updateRobotActionsforArm(RobotActions actions, Set<UserIntent> intent) {

        if (intent.contains(UserIntent.MANUAL_ELBOW)) {
            actions.cancelPresetArmActions();
            //actions.add(new MoveElbowAction(arm, 20 * driverControls.pivotJoystick() + arm.getElbowAngleInDegrees()));
            actions.add(new MoveElbowAction(arm, driverControls.pivotJoystick()));
        } else if (intent.contains(UserIntent.MANUAL_SLIDE)) {
            // first cancel any preset arm commands so we stop any preset slide/elbow moves
            actions.cancelPresetArmActions();
            //actions.add(new MoveSlideAction(arm, 0.5 * driverControls.slideMovement() + arm.getSlideExtension()));
            actions.add(new MoveSlideAction(arm,driverControls.slideMovement()));
        } else if (intent.contains(UserIntent.PRESET_SUBMERSIBLE_INTAKE)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.INTAKE_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_SAFE_DRIVING_POSITION)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.SAFE_DRIVING_POSITION));
        }
        if (intent.contains(UserIntent.MANUAL_DRIVE_NORMAL)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.SAFE_DRIVING_POSITION));
        }

        // check if you need to hold position for slide and/or elbow
        if (!actions.containsPresetArmActions()) {
            if (!intent.contains(UserIntent.MANUAL_ELBOW))
                actions.add(new HoldElbowAction(arm));
            if (!intent.contains(UserIntent.MANUAL_SLIDE) && !intent.contains(UserIntent.MANUAL_ELBOW)) {
                //we need to check for the elbow because moving the elbow causes the slide to move if it is
                //not less than 1 inch retracted, and the slide does not need to hold it's position
                //if it's extension is less than 1 inch
                actions.add(new HoldSlideAction(arm));
            }
        }

    }
    public static void updateRobotActionsforDriveTrain(RobotActions actions, Set<UserIntent> intent){
        if(intent.contains(UserIntent.SPEED_SWITCH)){
            actions.add(new ChangeSpeedAction(drivetrain));
        }
        if (intent.contains(UserIntent.IMU_RESET)) {
            actions.add(new ResetIMUAction(drivetrain));
        }
        if (intent.contains(UserIntent.MANUAL_DRIVE_NORMAL)) {
            double speedMultiplier = getSpeedMultiplier();
            if (speedMultiplier != DriveTrainSpeedMultiplier.NO_MULTIPLIER){
                actions.add(new MoveDriveTrainAction(drivetrain, speedMultiplier));
            } else {
                actions.add(new MoveDriveTrainAction(drivetrain));
            }
        }
        if (intent.contains(UserIntent.MANUAL_DRIVE_ADJUSTMENTS)) {
            actions.add(new MoveDriveTrainAction(drivetrain, DriveTrainSpeedMultiplier.SUPER_SLOW));
        }
        if(!intent.contains(UserIntent.MANUAL_DRIVE_NORMAL) && !intent.contains(UserIntent.MANUAL_DRIVE_ADJUSTMENTS)){
            actions.add(new StopDriveTrainAction(drivetrain));
        }
    }
    public static void updateRobotActionsForEndEffector(RobotActions actions, Set<UserIntent> intent){
        if (intent.contains(UserIntent.PRESET_SUBMERSIBLE_INTAKE)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.INTAKE_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserIntent.PRESET_SAFE_DRIVING_POSITION)){
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.SAFE_DRIVING_POSITION));
        }

        if (intent.contains(UserIntent.MANUAL_DRIVE_NORMAL)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.SAFE_DRIVING_POSITION));
        }

        if (intent.contains(UserIntent.INTAKE_FORWARD)){
            endEffector.activeIntakeForward();
        } else if (intent.contains(UserIntent.INTAKE_BACKWARD)){
            endEffector.activeIntakeBackward();
        } else if (!intent.contains(UserIntent.INTAKE_FORWARD) && !(intent.contains(UserIntent.INTAKE_BACKWARD))){
            endEffector.activeIntakeOff();
        }
    }
    private static double getSpeedMultiplier(){
        if (arm.getElbowAngleInDegrees() > 45){
            if (arm.getSlideExtension() > 10){
                return DriveTrainSpeedMultiplier.SUPER_SLOW;
            } else {
                return DriveTrainSpeedMultiplier.HALF_SPEED;
            }
        }
        return DriveTrainSpeedMultiplier.NO_MULTIPLIER;

    }
}
