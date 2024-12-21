package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.modules.arm.actions.HoldElbowAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.HoldSlideAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.HomeElbowAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.MoveElbowAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.MoveSlideAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.MoveToPresetPositionAction;
import org.firstinspires.ftc.teamcode.modules.arm.actions.SetArmPresetPosition;
import org.firstinspires.ftc.teamcode.modules.driverControl.actions.RumbleGamepadAction;
import org.firstinspires.ftc.teamcode.modules.driverControl.UserDirective;
import org.firstinspires.ftc.teamcode.modules.driveTrain.actions.ChangeSpeedAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainSpeedMultiplier;
import org.firstinspires.ftc.teamcode.modules.driveTrain.actions.MoveDriveTrainAction;
import org.firstinspires.ftc.teamcode.modules.driveTrain.actions.ResetIMUAction;
import org.firstinspires.ftc.teamcode.modules.driveTrain.actions.StopDriveTrainAction;
import org.firstinspires.ftc.teamcode.subsytems.activeIntake.ActiveIntakeDirection;
import org.firstinspires.ftc.teamcode.modules.endEffector.EndEffector;
import org.firstinspires.ftc.teamcode.modules.endEffector.EndEffectorMovement;
import org.firstinspires.ftc.teamcode.modules.endEffector.EndEffectorPresetPosition;
import org.firstinspires.ftc.teamcode.modules.endEffector.actions.MoveActiveIntakeAction;
import org.firstinspires.ftc.teamcode.modules.endEffector.actions.MoveEndEffectorThroughJoystickAction;
import org.firstinspires.ftc.teamcode.modules.endEffector.actions.MoveEndEffectorToPresetPositionAction;

import java.util.Set;

public class RobotCore {
    static Arm arm;
    static DriveTrain drivetrain;

    static DriverControls driverControls;
    static EndEffector endEffector;


    public static void initialize(DriverControls driverControls, DriveTrain drivetrain, Arm arm, EndEffector endEffector){
        RobotCore.arm = arm;
        RobotCore.drivetrain = drivetrain;
        RobotCore.driverControls = driverControls;
        RobotCore.endEffector = endEffector;
    }

    public static void updateRobotActionsforArm(RobotActions actions, Set<UserDirective> intent) {
        //Drivers requested to override safety rules
        boolean remove_arm_rules = false;
        if (intent.contains(UserDirective.REMOVE_ARM_RULES)){
            remove_arm_rules = true;
        }

        if (intent.contains(UserDirective.MANUAL_ELBOW)) {
            // first cancel any preset arm commands so we stop any preset slide/elbow moves
            actions.cancelPresetArmActions();
            actions.add(new MoveElbowAction(arm, driverControls.pivotJoystick()));
        } else if (intent.contains(UserDirective.MANUAL_SLIDE)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveSlideAction(arm, driverControls.slideMovement(), remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_SUBMERSIBLE_INTAKE)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.INTAKE_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.PRESET_SAFE_DRIVING_POSITION)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.SAFE_DRIVING_POSITION,remove_arm_rules));
        } else if (intent.contains(UserDirective.ELBOW_0)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.FLAT_ELBOW,remove_arm_rules));
        } else if (intent.contains(UserDirective.ELBOW_90)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.ASCENT_2_HANG,remove_arm_rules));
        } else if (intent.contains(UserDirective.HOME_ARM)){
            actions.cancelPresetArmActions();
            actions.add(new HomeElbowAction(arm));
        } else if (intent.contains(UserDirective.INTAKE_DOWN)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.INTAKE_DOWN,remove_arm_rules));
        }

        // check if you need to hold position for slide and/or elbow
        if (!actions.containsPresetArmActions()) {
            if (!intent.contains(UserDirective.MANUAL_ELBOW))
                actions.add(new HoldElbowAction(arm));
            if (!intent.contains(UserDirective.MANUAL_SLIDE) && !intent.contains(UserDirective.MANUAL_ELBOW)) {
                //we need to check for the elbow because moving the elbow causes the slide to move if it is
                //not less than 1 inch retracted, and the slide does not need to hold it's position
                //if it's extension is less than 1 inch
                actions.add(new HoldSlideAction(arm));
            }
        }


    }
    public static void updateRobotActionsforDriveTrain(RobotActions actions, Set<UserDirective> intent){
        if(intent.contains(UserDirective.SPEED_SWITCH)){
            actions.add(new ChangeSpeedAction(drivetrain));
        }
        if (intent.contains(UserDirective.IMU_RESET)) {
            actions.add(new ResetIMUAction(drivetrain));
        }
        if (intent.contains(UserDirective.MANUAL_DRIVE_NORMAL)) {
            double speedMultiplier = getSpeedMultiplier(intent);
            if (speedMultiplier != DriveTrainSpeedMultiplier.NO_MULTIPLIER){
                actions.add(new MoveDriveTrainAction(drivetrain, speedMultiplier));
            } else {
                actions.add(new MoveDriveTrainAction(drivetrain));
            }
        }
        if (intent.contains(UserDirective.MANUAL_DRIVE_ADJUSTMENTS)) {
            actions.add(new MoveDriveTrainAction(drivetrain, DriveTrainSpeedMultiplier.SUPER_SLOW));
        }
        if(!intent.contains(UserDirective.MANUAL_DRIVE_NORMAL) && !intent.contains(UserDirective.MANUAL_DRIVE_ADJUSTMENTS)){
            actions.add(new StopDriveTrainAction(drivetrain));
        }
        if(intent.contains(UserDirective.DRIVE_SWITCH)){
            if (drivetrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
                drivetrain.driveType = DriveTrain.DriveType.FIELD_CENTRIC;
            } else {
                drivetrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
            }
        }
    }
    public static void updateRobotActionsForEndEffector(RobotActions actions, Set<UserDirective> intent){
        if (intent.contains(UserDirective.PRESET_SUBMERSIBLE_INTAKE)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.INTAKE_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_SAFE_DRIVING_POSITION)){
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.SAFE_DRIVING_POSITION));
        } else if (intent.contains(UserDirective.WRIST_DOWN)){
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.DISPERSION));
        } else if (intent.contains(UserDirective.INTAKE_DOWN)){
            actions.add(new MoveEndEffectorToPresetPositionAction(arm, endEffector, EndEffectorPresetPosition.INTAKE_DOWN));
        }

        //gamepad control for the differential through dpad
        if (intent.contains(UserDirective.DIFF_UP)){
            actions.add(new MoveEndEffectorThroughJoystickAction(endEffector, EndEffectorMovement.UP));
        } else if (intent.contains(UserDirective.DIFF_DOWN)){
            actions.add(new MoveEndEffectorThroughJoystickAction(endEffector, EndEffectorMovement.DOWN));
        } else if (intent.contains(UserDirective.DIFF_LEFT)){
            actions.add(new MoveEndEffectorThroughJoystickAction(endEffector, EndEffectorMovement.LEFT));
        } else if (intent.contains(UserDirective.DIFF_RIGHT)){
            actions.add(new MoveEndEffectorThroughJoystickAction(endEffector, EndEffectorMovement.RIGHT));
        }

        if (intent.contains(UserDirective.INTAKE_FORWARD)){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.FORWARD));
        } else if (intent.contains(UserDirective.INTAKE_BACKWARD)){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.BACKWARD));
        } else if (!intent.contains(UserDirective.INTAKE_FORWARD) && !(intent.contains(UserDirective.INTAKE_BACKWARD))){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.OFF));
        }

        if (intent.contains(UserDirective.INTAKE_FORWARD) && endEffector.blockIn()){
            actions.add(new RumbleGamepadAction(driverControls));
        }

    }
    public static boolean updatePresetPositions(RobotActions actions, Set<UserDirective> directive){
        if (directive.contains(UserDirective.SET_DEPOSIT_BACK_BOTTOM)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_FRONT_BOTTOM)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_BACK_TOP)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_FRONT_TOP)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_SUBMERSIBLE_INTAKE)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.INTAKE_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_SAFE_DRIVING_POSITION)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.SAFE_DRIVING_POSITION, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_ELBOW_0)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.FLAT_ELBOW, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        if (directive.contains(UserDirective.SET_ELBOW_90)){
            actions.add(new SetArmPresetPosition(ArmPresetPosition.ASCENT_2_HANG, arm.getElbowAngleInDegrees(), arm.getSlideExtension()));
            return true;
        }
        return false;
    }
    private static double getSpeedMultiplier(Set<UserDirective> intent){
        if (!intent.contains(UserDirective.REMOVE_SPEED_RULES)){
            if (arm.getElbowAngleInDegrees() <= RobotConstants.ELBOWINTAKEANGLE){
                 return DriveTrainSpeedMultiplier.HALF_SPEED;
            } else if (RobotConstants.ELBOWINTAKEANGLE < arm.getElbowAngleInDegrees() && arm.getElbowAngleInDegrees() <= RobotConstants.ELBOWOUTTAKEANGLE){
                return DriveTrainSpeedMultiplier.NO_MULTIPLIER;
            } else {
                return  DriveTrainSpeedMultiplier.SUPER_SLOW;
            }
        } else {
            return DriveTrainSpeedMultiplier.NO_MULTIPLIER;
        }

    }
}
