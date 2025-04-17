package org.firstinspires.ftc.teamcode.teleop.stateModels;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.modules.arm.Arm;

public class ResetSlideEncoderStateModel {
    public enum SlideEncoderState{START,DEBOUNCE_WAIT,WAIT_FOR_SLIDES_TO_LEAVE}
    private static SlideEncoderState resetSlideEncoderState;
    static Arm arm;
    static ElapsedTime debounceTimer;
    public static void initialize(Arm arm){
        ResetSlideEncoderStateModel.arm = arm;
        debounceTimer = new ElapsedTime();
        resetSlideEncoderState = SlideEncoderState.START;
    }
    public static void execute(){
        switch(resetSlideEncoderState){
            case START:
                if (arm.isSlideTouchSensorPressed()){
                    debounceTimer.reset();
                    resetSlideEncoderState = SlideEncoderState.DEBOUNCE_WAIT;
                }
                break;
            case DEBOUNCE_WAIT:
                if (!arm.isSlideTouchSensorPressed()){
                    resetSlideEncoderState = SlideEncoderState.START;
                }
                if (debounceTimer.milliseconds() > 100){
                    arm.resetSlideEncoders();
                    arm.setSlidePower(0);
                    resetSlideEncoderState = SlideEncoderState.WAIT_FOR_SLIDES_TO_LEAVE;
                }
                break;
            case WAIT_FOR_SLIDES_TO_LEAVE:
                if (!arm.isSlideTouchSensorPressed()){
                    if (arm.getSlidePower() == 0){
                        arm.moveSlideToLength(arm.getSlideExtension());
                    }
                    resetSlideEncoderState = SlideEncoderState.START;
                }
                break;
        }
    }
}
