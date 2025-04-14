package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;

public class BringDepositBackToStartSafelyStateModel {
    private enum States{
        START,
        MOVE_PITCH,
        MOVE_ELBOW
    }
    private static States bringDepositBackToStartSafelyState;
    static Arm arm;
    static Wrist wrist;
    static ElapsedTime debounceTimer;
    public static boolean activate = false;
    public static void initialize(Arm arm, Wrist wrist){
        BringDepositBackToStartSafelyStateModel.arm = arm;
        BringDepositBackToStartSafelyStateModel.wrist = wrist;
        debounceTimer = new ElapsedTime();
        bringDepositBackToStartSafelyState = States.START;
    }
    public static void execute(){
        switch(bringDepositBackToStartSafelyState){
            case START:
                if (activate && FSMManager.getInstance().robotState == RobotState.READY_TO_DEPOSIT_IN_BUCKET){
                    FSMManager.getInstance().stopTransitions();
                    wrist.presetPositionPitch(StateModelParameters.DepositStateParameters.intermediatePitch);
                    debounceTimer.reset();
                    bringDepositBackToStartSafelyState = States.MOVE_PITCH;
                } else {
                    activate = false;
                }
                break;
            case MOVE_PITCH:
                if (debounceTimer.milliseconds() > 250){
                    arm.moveSlideToLength(0);
                    bringDepositBackToStartSafelyState = States.MOVE_ELBOW;
                }
                break;
            case MOVE_ELBOW:
                if (Math.abs(arm.getSlideExtension() - arm.getSlideTargetPositionInInches()) < RobotConstants.SLIDE_TOLERANCE){
                    activate = false;
                    FSMManager.getInstance().robotState = RobotState.START;
                    bringDepositBackToStartSafelyState = States.START;
                }
                break;
        }
    }
}
