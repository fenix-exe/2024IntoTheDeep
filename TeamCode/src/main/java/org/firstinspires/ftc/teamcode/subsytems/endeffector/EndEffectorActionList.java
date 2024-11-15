package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import java.util.ArrayList;
import java.util.List;

public class EndEffectorActionList {

    List<IEndEffectorAction> actions = new ArrayList<>();
    public EndEffectorActionList(){

    }
    public void add(IEndEffectorAction action){
        actions.add(action);
    }

    public boolean isEmpty(){
        return actions.isEmpty();
    }

    public EndEffectorState execute(){
        EndEffectorState activityState = EndEffectorState.IDLE.IDLE;
        for (int i=0; i<actions.size(); i++){
            IEndEffectorAction action = actions.get(i);
            /*if (action instanceof MoveToPresetPositionAction){
                activityState = ArmState.MOVING_TO_PRESET;
            }
            if (action instanceof MoveSlideAction || action instanceof MoveElbowAction){
                activityState = ArmState.MANUAL_MOVE;
            }*/
            actions.get(i).execute();
        }
        return activityState;
    }
}
