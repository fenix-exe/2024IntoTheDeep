package org.firstinspires.ftc.teamcode.subsytems.arm;

import java.util.ArrayList;
import java.util.List;

public class ArmActionList {

    List<IArmAction> actions = new ArrayList<>();
    public ArmActionList(){

    }
    public void add(IArmAction action){
        actions.add(action);
    }

    public boolean isEmpty(){
        return actions.isEmpty();
    }

    public void clear() {
        actions.clear();
    }

    public ArmState execute(){
        ArmState activityState = ArmState.IDLE;
        for (int i=0; i<actions.size(); i++){
            IArmAction action = actions.get(i);
            if (action instanceof MoveToPresetPositionAction){
                activityState = ArmState.MOVING_TO_PRESET;
            }
            if (action instanceof MoveSlideAction || action instanceof MoveElbowToPresetAction){
                activityState = ArmState.MANUAL_MOVE;
            }
            actions.get(i).execute();
        }
        return activityState;
    }
}
