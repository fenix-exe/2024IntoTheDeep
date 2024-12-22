package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.util.LoggerUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Vector;

public class RobotActions {
    
    HashSet<IRobotAction> actions = new HashSet<>();
    
    public RobotActions(){
        
    }
    
    public void cancelPresetArmActions(){
        for (IRobotAction action : actions) {
            if(action.getActionType() == IRobotAction.ActionType.ARM_MOVE_TO_PRESET){
                action.cancel();
            }
        }
    }

    public boolean containsPresetArmActions(){
        for (IRobotAction action : actions) {
            if(action.getActionType() == IRobotAction.ActionType.ARM_MOVE_TO_PRESET){
                return true;
            }
        }
        return false;
    }

    public boolean containsPresetEnndEffectorActions(){
        for (IRobotAction action : actions) {
            if(action.getActionType() == IRobotAction.ActionType.END_EFFECTOR_MOVE_TO_PRESET){
                return true;
            }
        }
        return false;
    }

    public void add(IRobotAction action){
        actions.add(action);
    }

    public void execute() {
        for (IRobotAction action : actions) {
            action.execute();
        }
    }

    public void removeCompleteAndCancelled(){
        HashSet<IRobotAction> actionsToRemove = new HashSet<>();
        for (IRobotAction action : actions) {
            if(action.isCancelled() || action.isComplete()){
                actionsToRemove.add(action);
            }
        }
        for (IRobotAction action : actionsToRemove) {
            actions.remove(action);
            LoggerUtil.info(action.toString());
        }
    }

    public String toString() {
        Vector<String> action_types = new Vector<String>();
        for (IRobotAction action : actions) {
            action_types.add(action.getClass().getSimpleName());
            //action_types.add(action.toString());
        }
        Collections.sort(action_types);
        return String.join(",", action_types);
    }
    
}
