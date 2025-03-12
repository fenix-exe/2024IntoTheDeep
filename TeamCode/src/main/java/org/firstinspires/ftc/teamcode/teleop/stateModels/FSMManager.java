package org.firstinspires.ftc.teamcode.teleop.stateModels;



import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

import java.util.ArrayList;

public class FSMManager {
    public static RobotState robotState;
    static ArrayList<IStateTransition> stateTransitions = new ArrayList<>();
    public static void initialize(Wrist wrist, Claw claw, Arm arm, DriveTrain driveTrain, DriverControls driverControls, ColorSensor color, LinearActuator linearActuator){
        robotState = RobotState.START;
        stateTransitions.add(new GoToIntakeStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new GrabSampleStateTransition(wrist, claw, arm, driveTrain, driverControls));
        stateTransitions.add(new GrabFailedStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new LeaveSubmersibleStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new GoToDepositStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new DepositSampleIntoObservationZoneStateTransition(wrist,claw,arm,driverControls));
        stateTransitions.add(new GoToGrabSpecimenPositionStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new GrabSpecimenStateTransition(wrist,claw,arm,driveTrain,driverControls,color));
        stateTransitions.add(new GrabFailedForSpecimensStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new GoToClipSpecimenStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new LetGoOfClipStateTransition(wrist, claw, arm, driverControls));
        stateTransitions.add(new HangStateTransition(wrist, claw, arm, driverControls,linearActuator));
    }
    public static void execute(){
        for(int i = 0; i < stateTransitions.size(); i++){
            stateTransitions.get(i).execute();
        }
    }
    public static void stopTransitions(){
        for (int i = 0; i<stateTransitions.size(); i++){
            stateTransitions.get(i).reset();
        }
    }
    public static void setRobotStateToStart(){
        robotState = RobotState.START;
    }
    public static boolean isAtStart(){
        return robotState == RobotState.START;
    }

}
