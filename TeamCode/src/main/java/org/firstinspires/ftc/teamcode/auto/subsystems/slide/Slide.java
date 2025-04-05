package org.firstinspires.ftc.teamcode.auto.subsystems.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.SlideMessage;
import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.StatusMessage;
import org.firstinspires.ftc.teamcode.common.CommonSlide;


import androidx.annotation.NonNull;

public class Slide extends CommonSlide {

    /*
    * This class allows for control of the slide using inches in autonomous
     */


    public DcMotorEx leftSlideMotor;
    public DcMotorEx rightSlideMotor;
    public RevTouchSensor homingSwitch;
    private final DownsampledWriter slideWriter;
    private final DownsampledWriter slideStatusWriter;



    public Slide(DcMotorEx leftSlideMotor, DcMotorEx rightSlideMotor, RevTouchSensor homingSwitch){
        super(leftSlideMotor, rightSlideMotor, homingSwitch);
        this.homingSwitch = homingSwitch;
        this.leftSlideMotor = leftSlideMotor;
        this.rightSlideMotor = rightSlideMotor;
        slideWriter = new DownsampledWriter("SLIDE INFO", 50_000_000);
        slideStatusWriter = new DownsampledWriter("SLIDE STATUS", 50_000_000);
    }

    public void setSlideExtensionLengthAndSpeed(double lengthInInches, double speed){
        int targetPosition = inchesToTicks(lengthInInches);
        rightSlideMotor.setTargetPosition(targetPosition);
        leftSlideMotor.setTargetPosition(targetPosition);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setPower(speed);
        leftSlideMotor.setPower(speed);
    }


    /* this action sets slide motor position using inches
    * finishes when slide is within 0.5 inches of the position
    * slides continue to stay at its position when the action finishes
     */
    public class slideControl implements Action {
        private final double targetPos;
        private final double speed;
        private boolean initialized = false;
        private double time;

        slideControl(double targetPos, double speed){
            this.targetPos = targetPos;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized && ((ticksToInches(leftSlideMotor.getCurrentPosition()) - 0.5 < getSlideExtensionInInches()) || !(getSlideExtensionInInches() < ticksToInches(leftSlideMotor.getCurrentPosition()) + 0.5))) {
            if (!initialized) {
                time = System.currentTimeMillis();
                slideStatusWriter.write(new StatusMessage("SLIDES MOVING"));
                initialized=true;
            }
            setSlideExtensionLengthAndSpeed(targetPos, speed);

            if (homingSwitch.isPressed() && System.currentTimeMillis()>=time+500){
                leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setSlideExtensionLengthAndSpeed(targetPos, speed);
                slideStatusWriter.write(new StatusMessage("SLIDES RESET"));
                time = System.currentTimeMillis();
            }

            slideWriter.write(new SlideMessage(getSlideExtensionInInches(), ticksToInches(leftSlideMotor.getCurrentPosition()), leftSlideMotor.getCurrent(CurrentUnit.MILLIAMPS), rightSlideMotor.getCurrent(CurrentUnit.MILLIAMPS)));

            return !(targetPos - 0.5 < getSlideExtensionInInches()) || !(getSlideExtensionInInches() < targetPos + 0.5);
            } else {
                slideWriter.write(new SlideMessage(getSlideExtensionInInches(), ticksToInches(leftSlideMotor.getCurrentPosition()), leftSlideMotor.getCurrent(CurrentUnit.MILLIAMPS), rightSlideMotor.getCurrent(CurrentUnit.MILLIAMPS)));
                slideStatusWriter.write(new StatusMessage("NOT REACHED PREVIOUS TARGET POS"));
                return true;}
        }
    }
    public Action slideControl(double targetPos, double speed){
        return new slideControl(targetPos, speed);
    }


}
