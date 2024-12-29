package org.firstinspires.ftc.teamcode.subsytems.arm;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

public class Arm {
    Slide slide;
    Elbow elbow;
    int physicalMaxExtension;
    double MaxSlideExtensionInches;


    public Arm(Slide slide, Elbow elbow, int initialTopHeight){
        this.slide = slide;
        this.elbow = elbow;
        this.physicalMaxExtension = initialTopHeight;
    }

    public void moveSlide(double slideMovement, boolean remove_arm_rules) {
        int max_extension = getSlideMaxLengthIn42Inches(getElbowAngleInTicks());
        slide.joystickControl(slideMovement, max_extension,remove_arm_rules);
    };

    public void moveElbow(double elbowMovement, boolean remove_arm_rules){
        if ((slide.getSlideExtensionInInches() > 1 && !remove_arm_rules) && !(elbow.getElbowAngle() < 20 && slide.getSlideExtensionInInches() >= 3)){
            slide.setSlideExtensionLength(0);
        } else {
            double power;
            if (elbow.getElbowTicks() > elbow.topPosition - 100 && elbowMovement > 0){ //top limit
                power = 0;
            } else if (elbow.getElbowAngle() < -5
                    && elbowMovement < 0){ //bottom limit is dependent on slides
                power = 0;
            } else {
                power = elbowMovement;
            }
            elbow.elbowJoystick(power);
        }
    }

    public int getSlideMaxLengthIn42Inches(int angleInTicks){
        int topHeight;
        double theta = elbow.ticksToDegrees(angleInTicks);
        if (theta != 90) {
            MaxSlideExtensionInches = 31/(Math.cos(Math.toRadians(theta)));
        } else {
            MaxSlideExtensionInches = 10^44;
        }
        int MaxSlideExtensionEncoderTicks = slide.inchesToTicksPivotPoint(MaxSlideExtensionInches);
        if (MaxSlideExtensionEncoderTicks < physicalMaxExtension){
            topHeight = MaxSlideExtensionEncoderTicks;
        } else {
            topHeight = physicalMaxExtension;
        }
        return (int) Math.floor(Math.abs(topHeight));
    }
    public int getSlideLengthAllowed(double angleInDegrees){
        if (angleInDegrees > 75){
            return 700;
        } else if (angleInDegrees > 45 && angleInDegrees < 75){
            return 1000;
        } else {
            return 5000;
        }
    }
    public void resetEncoders(){
        elbow.resetEncoder();
        slide.resetEncoder();
        elbow.setTargetAngle(0);
        slide.setSlideExtensionLengthInTicks(0);
    }
    public void setElbowPower(double power){
        elbow.setElbowPower(power);
    }
    public int currentAllowedMaxExtensionLength(){
        return Math.min(getSlideMaxLengthIn42Inches(elbow.getElbowTicks()), getSlideLengthAllowed(elbow.getElbowAngle()));
    }
    public int currentAllowedMaxExtensionLength(int angleInTicks){
        return Math.min(getSlideMaxLengthIn42Inches(angleInTicks), getSlideLengthAllowed(elbow.ticksToDegrees(angleInTicks)));
    }
    public boolean doesSlideNeedToRetract(int angleInTicks){
        return slide.getSlidePosition() > Math.min(getSlideMaxLengthIn42Inches(angleInTicks), getSlideLengthAllowed(elbow.ticksToDegrees(angleInTicks)));
    }
    public double getElbowAngleInDegrees(){
        return elbow.getElbowAngle();
    }
    public int getElbowAngleInTicks(){return elbow.getElbowTicks();}
    public void setSlideLengthInInches(double inches){
        slide.setSlideExtensionLength(inches);
    }

    public void setElbowAngleInDegrees(double angle){
        elbow.setTargetAngle(angle);
    }
    public void holdElbow(){
        elbow.holdPosition();
    }
    public void holdSlide(){slide.holdPosition();}

    public void moveToPresetPosition(ArmPresetPosition position, boolean manual_override_arm_rules){
        //if elbow needs to be moved to reach position, retract slides fully
       if (manual_override_arm_rules || (elbow.getElbowAngle() < 20 && !(position.elbowAngle > 20))){
           slide.setSlideExtensionLength(position.slideLength);
           elbow.setTargetAngle(position.elbowAngle);
       } else {
           if (position.elbowAngle > 20 && elbow.getElbowAngle() < 23){
               elbow.setTargetAngle(25);
           } else if (Math.abs(elbow.getElbowAngle() - position.elbowAngle) > 3 && slide.getSlideExtensionInInches() > 1) {
               slide.setSlideExtensionLength(0);
           } else {
               //move elbow first
               if (Math.abs(elbow.getElbowAngle() - position.elbowAngle) > 3) {
                   elbow.setTargetAngle(position.elbowAngle);
               } else {
                   // now move slide
                   slide.setSlideExtensionLength(position.slideLength);
               }
           }
       }
    }

    public boolean isArmAtPresetPosition(ArmPresetPosition position){
        return Math.abs(elbow.getElbowAngle() - position.elbowAngle) < 3 &&  // angle is within 3 degrees of target
                Math.abs(slide.getSlideExtensionInInches() - position.slideLength) < 1; // slide is within 1 inch of target
    }

    public boolean isBusy(){
        return slide.isBusy() || elbow.isBusy();
    }
    public double getSlideExtension(){
        return slide.getSlideExtensionInInches();
    }


}
