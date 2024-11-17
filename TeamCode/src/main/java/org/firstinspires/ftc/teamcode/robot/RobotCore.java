package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsytems.DriverControls;
import org.firstinspires.ftc.teamcode.subsytems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsytems.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.arm.HoldElbowAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.HoldSlideAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveElbowAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveSlideAction;
import org.firstinspires.ftc.teamcode.subsytems.arm.MoveToPresetPositionAction;
import org.firstinspires.ftc.teamcode.subsytems.driverControl.UserDirective;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.ChangeSpeedAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.DriveTrainSpeedMultiplier;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.MoveDriveTrainAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.ResetIMUAction;
import org.firstinspires.ftc.teamcode.subsytems.drivetrain.StopDriveTrainAction;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.ActiveIntakeDirection;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffector;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorMovement;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.MoveActiveIntakeAction;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.MoveEndEffectorThroughJoystick;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.MoveEndEffectorToPresetPositionAction;

import java.util.HashSet;
import java.util.Set;

public class RobotCore {
    static Arm arm;
    static DriveTrain drivetrain;

    static DriverControls driverControls;
    static EndEffector endEffector;
    public static boolean remove_arm_rules = false;
    public static Gamepad gamepad2;

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

    public static void updateRobotActionsforArm(RobotActions actions, Set<UserDirective> intent) {

        if (intent.contains(UserDirective.MANUAL_ELBOW)) {
            actions.cancelPresetArmActions();
            //actions.add(new MoveElbowAction(arm, 20 * driverControls.pivotJoystick() + arm.getElbowAngleInDegrees()));
            actions.add(new MoveElbowAction(arm, driverControls.pivotJoystick()));
        } else if (intent.contains(UserDirective.MANUAL_SLIDE)) {
            // first cancel any preset arm commands so we stop any preset slide/elbow moves
            actions.cancelPresetArmActions();
            //actions.add(new MoveSlideAction(arm, 0.5 * driverControls.slideMovement() + arm.getSlideExtension()));
            actions.add(new MoveSlideAction(arm,driverControls.slideMovement()));
        } else if (intent.contains(UserDirective.PRESET_SUBMERSIBLE_INTAKE)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.INTAKE_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_SAFE_DRIVING_POSITION)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.SAFE_DRIVING_POSITION));
        } else if (intent.contains(UserDirective.ELBOW_0)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.FLAT_ELBOW));
        } else if (intent.contains(UserDirective.ELBOW_90)){
            actions.cancelPresetArmActions();
            actions.add(new MoveToPresetPositionAction(arm, ArmPresetPosition.ASCENT_2_HANG));
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

        if (intent.contains(UserDirective.REMOVE_ARM_RULES)){
            remove_arm_rules = true;
        } else {
            remove_arm_rules = false;
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
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.INTAKE_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_BACK_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_TOP)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_DEPOSIT_FRONT_BOTTOM)) {
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION));
        } else if (intent.contains(UserDirective.PRESET_SAFE_DRIVING_POSITION)){
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.SAFE_DRIVING_POSITION));
        } else if (intent.contains(UserDirective.WRIST_DOWN)){
            actions.add(new MoveEndEffectorToPresetPositionAction(endEffector, EndEffectorPresetPosition.INTAKE_DOWN_ON_GROUND));
        }

        //gamepad control for the differential through dpad
        if (intent.contains(UserDirective.DIFF_UP)){
            actions.add(new MoveEndEffectorThroughJoystick(endEffector, EndEffectorMovement.UP));
        } else if (intent.contains(UserDirective.DIFF_DOWN)){
            actions.add(new MoveEndEffectorThroughJoystick(endEffector, EndEffectorMovement.DOWN));
        } else if (intent.contains(UserDirective.DIFF_LEFT)){
            actions.add(new MoveEndEffectorThroughJoystick(endEffector, EndEffectorMovement.LEFT));
        } else if (intent.contains(UserDirective.DIFF_RIGHT)){
            actions.add(new MoveEndEffectorThroughJoystick(endEffector, EndEffectorMovement.RIGHT));
        }

        if (intent.contains(UserDirective.INTAKE_FORWARD)){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.FORWARD));
        } else if (intent.contains(UserDirective.INTAKE_BACKWARD)){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.BACKWARD));
        } else if (!intent.contains(UserDirective.INTAKE_FORWARD) && !(intent.contains(UserDirective.INTAKE_BACKWARD))){
            actions.add(new MoveActiveIntakeAction(endEffector, ActiveIntakeDirection.OFF));
        }

        if (intent.contains(UserDirective.INTAKE_FORWARD) && endEffector.blockIn()){
            gamepad2.rumble(10);
        }

    }
    public static void updatePresetPositions(Set<UserDirective> directive){
        if (directive.contains(UserDirective.SET_DEPOSIT_BACK_BOTTOM)){
            ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_FRONT_BOTTOM)){
            ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_BACK_TOP)){
            ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
        if (directive.contains(UserDirective.SET_DEPOSIT_FRONT_TOP)){
            ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
        if (directive.contains(UserDirective.SET_SUBMERSIBLE_INTAKE)){
            ArmPresetPosition.INTAKE_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
        if (directive.contains(UserDirective.SET_SAFE_DRIVING_POSITION)){
            ArmPresetPosition.SAFE_DRIVING_POSITION = new ArmPresetPosition(arm.getElbowAngleInDegrees(), arm.getSlideExtension());
        }
    }
    private static double getSpeedMultiplier(Set<UserDirective> intent){
        if (!intent.contains(UserDirective.REMOVE_SPEED_RULES)){
            if (arm.getElbowAngleInDegrees() <= 30){
                 return DriveTrainSpeedMultiplier.HALF_SPEED;
            } else if (30 < arm.getElbowAngleInDegrees() && arm.getElbowAngleInDegrees() <= 60){
                return DriveTrainSpeedMultiplier.NO_MULTIPLIER;
            } else {
                return  DriveTrainSpeedMultiplier.SUPER_SLOW;
            }
        } else {
            return DriveTrainSpeedMultiplier.NO_MULTIPLIER;
        }

    }
}
