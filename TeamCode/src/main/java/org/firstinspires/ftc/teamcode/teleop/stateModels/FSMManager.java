package org.firstinspires.ftc.teamcode.teleop.stateModels;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriveControlMap;
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
    public RobotState robotState;
    Arm arm;
    Wrist wrist;
    MoveToLeaveSubmersibleStateTransition moveToLeaveSubmersibleStateTransition;
    GrabSpecimenStateTransition grabSpecimenStateTransition;
    ArrayList<IStateTransition> stateTransitions;

    private static FSMManager instance = null;

    private FSMManager() {
        robotState = RobotState.START;
        stateTransitions = new ArrayList<>();
    }

    public static FSMManager getInstance() {
        if(instance == null){
            instance = new FSMManager();
        }
        return instance;
    }

    public static FSMManager getInstance(boolean forceNewInstance) {
        if(forceNewInstance){
            instance = new FSMManager();
        }
        return instance;
    }

    public void initialize(Wrist wrist, IIntake intake, Arm arm, IDriveTrain driveTrain, DriveControlMap driverControls, ColorSensor color, LinearActuator linearActuator, Alliance alliance, boolean colorSensorConnected){
        stateTransitions.clear();
        this.moveToLeaveSubmersibleStateTransition = new MoveToLeaveSubmersibleStateTransition(wrist, intake, arm, driveTrain, driverControls, colorSensorConnected?color:null, alliance);
        this.grabSpecimenStateTransition = new GrabSpecimenStateTransition(wrist,intake,arm,driveTrain,driverControls,colorSensorConnected?color:null);
        stateTransitions.add(new GoToIntakeStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(this.moveToLeaveSubmersibleStateTransition);
        stateTransitions.add(new GrabFailedStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new GoToDepositStateTransition(wrist, arm, driverControls));
        stateTransitions.add(new DepositSampleIntoObservationZoneStateTransition(wrist,arm,driverControls));
        stateTransitions.add(new GoToGrabSpecimenPositionStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(this.grabSpecimenStateTransition);
        stateTransitions.add(new GrabFailedForSpecimensStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new GoToClipSpecimenStateTransition(wrist, arm, driverControls));
        stateTransitions.add(new LetGoOfClipStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new HangStateTransition(wrist, arm, driverControls,linearActuator));
        stateTransitions.add(new EnterSubmersibleStateTransition(wrist,intake,driverControls));
        stateTransitions.add(new LeaveDepositStateTransition(wrist, intake, arm, driverControls));
        stateTransitions.add(new MoveElbowUpToDepositionStateTransition(wrist,arm,driverControls));
        stateTransitions.add(new EnterIntakeStateFromElbowUpStateTransition(wrist,intake,arm,driverControls));
        stopTransitions();
        setRobotStateToStart();
        this.arm = arm;
        this.wrist = wrist;
    }
    public void execute(){
        for(int i = 0; i < stateTransitions.size(); i++){
            stateTransitions.get(i).execute();
        }
    }
    public void stopTransitions(){
        for (int i = 0; i<stateTransitions.size(); i++){
            stateTransitions.get(i).reset();
        }
    }
    public void setRobotStateToStart(){
        robotState = RobotState.START;
    }
    public boolean isAtStart(){
        return robotState == RobotState.START;
    }
    public void updateBasedOnColorSensorStatus(){
        grabSpecimenStateTransition.color = null;
        moveToLeaveSubmersibleStateTransition.colorSensor = null;
    }
    public void debug(Telemetry telementry){
        StringBuilder strBuilder = new StringBuilder();
        for (int i = 0; i<stateTransitions.size(); i++){
            if(stateTransitions.get(i).inProgress()){
                strBuilder.append(i);
                strBuilder.append(",");
            }
        }
        telementry.addLine("FSM states in progress: "+strBuilder.toString());
        moveToLeaveSubmersibleStateTransition.debug(telementry);
    }

}
