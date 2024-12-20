package org.firstinspires.ftc.teamcode.modules.arm;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;

import java.util.HashMap;

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
    }

    public void moveElbow(double elbowMovement){
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

    public int getSlideMaxLengthIn42Inches(int angleInTicks){
        int topHeight;
        double theta = elbow.ticksToDegrees(angleInTicks);
        if (theta != 90) {
            MaxSlideExtensionInches = 31/(Math.cos(Math.toRadians(theta)));
        } else {
            MaxSlideExtensionInches = 10^44;
        }
        int MaxSlideExtensionEncoderTicks = slide.inchesToTicksPivotPoint(MaxSlideExtensionInches);
        topHeight = Math.min(physicalMaxExtension, MaxSlideExtensionEncoderTicks);
        return (int) Math.floor(Math.abs(topHeight));
    }
    public void resetEncoders(){
        elbow.resetEncoder();
        slide.resetEncoder();
        elbow.setTargetAngle(0);
        slide.setSlideExtensionLength(0);
    }
    public void setElbowPower(double power){
        elbow.setElbowPower(power);
    }
    public double getElbowAngleInDegrees(){
        return elbow.getElbowAngle();
    }
    public int getElbowAngleInTicks(){return elbow.getElbowTicks();}
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
            } else if (Math.abs(elbow.getElbowAngle() - position.elbowAngle) > 5 && slide.getSlideExtensionInInches() > 5) {
                slide.setSlideExtensionLength(0);
            } else {
                //move elbow first
                if (Math.abs(elbow.getElbowAngle() - position.elbowAngle) > 5) {
                    elbow.setTargetAngle(position.elbowAngle);
                } else {
                    // now move slide
                    elbow.setTargetAngle(position.elbowAngle);
                    slide.setSlideExtensionLength(position.slideLength);
                }
            }
        }
    }

    public boolean isArmAtPresetPosition(ArmPresetPosition position){
        return Math.abs(elbow.getElbowAngle() - position.elbowAngle) < 3 &&  // angle is within 3 degrees of target
                Math.abs(slide.getSlideExtensionInInches() - position.slideLength) < 1; // slide is within 1 inch of target
    }
    public double getSlideExtension(){
        return slide.getSlideExtensionInInches();
    }

    public HashMap getDebugInfo() {

        HashMap debugInfo = new HashMap<>();
        debugInfo.put("Slide Extension", this.getSlideExtension());
        debugInfo.put("Slide Limit", this.getSlideMaxLengthIn42Inches(this.getElbowAngleInTicks()));
        debugInfo.put("Slide Power", this.slide.slideMotor.getPower());
        debugInfo.put("Slide Current", this.slide.slideMotor.getCurrent(CurrentUnit.MILLIAMPS));
        debugInfo.put("Elbow Angle", this.getElbowAngleInDegrees());
        debugInfo.put("Elbow Power", this.elbow.elbowMotor.getPower());
        debugInfo.put("Elbow Current", this.elbow.elbowMotor.getCurrent(CurrentUnit.MILLIAMPS));
        return debugInfo;
    }


}
