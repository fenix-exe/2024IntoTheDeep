package org.firstinspires.ftc.teamcode.subsytems.drivetrain;

import org.firstinspires.ftc.teamcode.robot.RobotActivityState;

import java.util.ArrayList;
import java.util.List;

public class DriveTrainActionList {
    List<IDriveTrainAction> actions = new ArrayList<>();
    public DriveTrainActionList(){

    }
    public void add(IDriveTrainAction action){
        actions.add(action);
    }
    public DriveTrainState execute(){
        DriveTrainState activityState = DriveTrainState.IDLE;
        for (int i=0; i<actions.size(); i++){
            IDriveTrainAction action = actions.get(i);
            if (action instanceof MoveDriveTrainAction){
                activityState = DriveTrainState.MOVING;
            }
            actions.get(i).execute();
        }
        return activityState;
    }
}
