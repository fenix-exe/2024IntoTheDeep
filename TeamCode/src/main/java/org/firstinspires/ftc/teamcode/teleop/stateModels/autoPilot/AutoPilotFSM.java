package org.firstinspires.ftc.teamcode.teleop.stateModels.autoPilot;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.stateModels.FSMManager;
import org.firstinspires.ftc.teamcode.teleop.stateModels.IStateTransition;
import org.firstinspires.ftc.teamcode.teleop.stateModels.RobotState;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;

import java.util.ArrayList;

public class AutoPilotFSM {
    public static AutoPilotState autoPilotState;
    private static boolean activatedTransitions = true;
    private static boolean goToClip = false;
    private static boolean depositClip = false;
    private static boolean goToPickupClip = false;
    //static IDriveTrain driveTrain;
    static DriverControls driverControls;
    static ArrayList<IStateTransition> stateTransitions = new ArrayList<>();
    public static void initialize(IDriveTrain driveTrain, DriverControls driverControls, Telemetry telemetry){
        AutoPilotFSM.driverControls = driverControls;
        //AutoPilotFSM.driveTrain = driveTrain;
        autoPilotState = AutoPilotState.START;
        stateTransitions.add(new DriveToBucketStateTransition(new Pose2d(11.8,45,Math.toRadians(-45)),driveTrain, telemetry));
        stateTransitions.add(new DriveToSubmersibleStateTransition(new Pose2d(65.8,31,Math.toRadians(-90)),driveTrain, telemetry));
        stateTransitions.add(new DriveToClipFSM(new Pose2d(10,10,0), new Pose2d(15,10,0), new Pose2d(0,0,0), driveTrain, telemetry));
        stopTransitions();
    }
    public static void execute(){
        //if (driverControls.activateAutoDrive()) {
            activatedTransitions = true;
            for(int i = 0; i < stateTransitions.size(); i++){
                stateTransitions.get(i).execute();
            }
        //} else if (activatedTransitions){
            activatedTransitions = false;
            //driveTrain.stopFollowing();
            stopTransitions();
        //}
    }
    public static void stopTransitions(){
        for (int i = 0; i<stateTransitions.size(); i++){
            stateTransitions.get(i).reset();
        }
        autoPilotState = AutoPilotState.START;
    }
    public static boolean isSafeToGoToDeposit(){
        /*if (driverControls.activateAutoDrive()){
            Pose2d pose = driveTrain.getCurrentPose();
            if(!(Math.abs(pose.component1().x-66.5)<24 && Math.abs(pose.component1().x-66.5) > 14.5 && Math.abs(pose.component1().y-7.25) < 24)) {
                return true;
            }
        }*/
        return false;
    }
    public static boolean returnToSubmersible(){
        //if (driverControls.activateAutoDrive()){
                return FSMManager.getInstance().robotState == RobotState.READY_TO_INTAKE_SAMPLE;
        //}
        //return false;
    }
    public static void setGoToClip(boolean goToClip){
        AutoPilotFSM.goToClip = goToClip;
    }
    public static boolean getGoToClip(){
        return goToClip;
    }
    public static void setDepositClip(boolean depositClip){
        AutoPilotFSM.depositClip = depositClip;
    }
    public static boolean getDepositClip(){
        return depositClip;
    }
    public static void setGoToPickupClip(boolean pickupClip){
        AutoPilotFSM.goToPickupClip = pickupClip;
    }
    public static boolean getGoToPickupClip(){
        return goToPickupClip;
    }
}
