package org.firstinspires.ftc.teamcode.teleop.stateModels;



import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.IIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator.LinearActuator;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.util.Alliance;

import java.util.ArrayList;

public class FSMManager {
    public static RobotState robotState;
    static ArrayList<IStateTransition> stateTransitions = new ArrayList<>();
    public static void initialize(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriverControls driverControls, ColorSensor color, LinearActuator linearActuator, Alliance alliance, boolean colorSensorConnected){
        robotState = RobotState.START;
        stateTransitions.add(new GoToIntakeStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new MoveToLeaveSubmersibleStateTransition(wrist, intake, arm, driveTrain, driverControls, colorSensorConnected?color:null, alliance));
        stateTransitions.add(new GrabFailedStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new GoToDepositStateTransition(wrist, arm, driverControls));
        stateTransitions.add(new DepositSampleIntoObservationZoneStateTransition(wrist,arm,driverControls));
        stateTransitions.add(new GoToGrabSpecimenPositionStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new GrabSpecimenStateTransition(wrist,intake,arm,driveTrain,driverControls,colorSensorConnected?color:null));
        stateTransitions.add(new GrabFailedForSpecimensStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new GoToClipSpecimenStateTransition(wrist, arm, driverControls));
        stateTransitions.add(new LetGoOfClipStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new HangStateTransition(wrist, arm, driverControls,linearActuator));
        stateTransitions.add(new EnterSubmersibleStateTransition(wrist,intake,driverControls));
        stateTransitions.add(new LeaveDepositStateTransition(wrist, intake, arm, driverControls));
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
