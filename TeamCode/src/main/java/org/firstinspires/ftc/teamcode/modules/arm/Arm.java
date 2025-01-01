package org.firstinspires.ftc.teamcode.modules.arm;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;

import java.util.HashMap;

public class Arm {
    Slide slide;
    public Elbow elbow;

    public Arm(Slide slide, Elbow elbow){
        this.slide = slide;
        this.elbow = elbow;
        ArmSpeedController.slide = slide;
    }

    public void moveSlide(double slideMovement, boolean remove_arm_rules) {
        //max_extension already includes tolerance
        double max_extension = getMaximumSlideExtensionAllowedInInches();
        double power;
        if (!remove_arm_rules){
            if (slide.getSlideExtensionInInches() > max_extension
                    && slideMovement > 0){ //top limit
                power = 0;
            } else if (slide.getSlideExtensionInInches() < RobotConstants.SLIDE_TOLERANCE
                    && slideMovement < 0){ //bottom limit
                power = 0;
            } else {
                power = slideMovement;
            }
        } else {
            power = slideMovement;
        }
        slide.joystickControl(power);
    }

    public void moveElbow(double elbowMovement){
            double power;
            if (elbow.getElbowTicks() > elbow.topPosition - ArmConstants.ELBOWTICKSTOLERANCE
                    && elbowMovement > 0){ //top limit
                power = 0;
            } else if (elbow.getElbowAngle() < ArmConstants.ELBOWBOTTOMANGLE
                    && elbowMovement < 0){ //bottom limit
                power = 0;
            } else {
                power = elbowMovement;
            }
            elbow.elbowJoystick(power);
    }

    private double getMaximumSlideExtensionAllowedInInches(){
        double theta = elbow.getElbowAngle();
        double MaxSlideExtensionInches = RobotConstants.PHYSICAL_MAX_EXTENSION_IN_INCHES;
        if (!(Math.abs(90-theta) < 1)) { //tolerance of 1 degree around 90 degrees, I cannot compare double directly to int
            MaxSlideExtensionInches = Math.min(RobotConstants.PHYSICAL_MAX_EXTENSION_IN_INCHES,
                    ArmConstants.MAXSLIDEEXTENSIONLENGTHINCHES/(Math.cos(Math.toRadians(theta))));
        }
        //add 1 inch safety margin
        MaxSlideExtensionInches -= RobotConstants.SLIDE_TOLERANCE;
        return MaxSlideExtensionInches;
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
    public void holdArm(){
        holdElbow();
        holdSlide();
    }
    public double getElbowTargetPositionInDegrees(){
        return elbow.ticksToDegrees(elbow.elbowMotor.getTargetPosition());
    }
    public double getSlideTargetPositionInInches(){
        return slide.ticksToInches(slide.slideMotor.getTargetPosition());
    }

    public void moveToPresetPosition(ArmPresetPosition position, boolean manual_override_arm_rules){
        //if elbow is not at preset position, move elbow to preset position
        if ((Math.abs(elbow.getElbowAngle() - position.elbowAngle) > ArmConstants.ELBOWPRESETTOLERANCE) || manual_override_arm_rules){ //checsk to see if the elbow is not within a tolerance of the target
            elbow.setTargetAngleAndSpeed(position.elbowAngle, ArmSpeedController.getElbowPowerLimit());
        } else {
            // if elbow is at preset position, move slide to preset position
            elbow.setTargetAngleAndSpeed(position.elbowAngle, ArmSpeedController.getElbowPowerLimit());
            slide.setSlideExtensionLength(position.slideLength);
        }
    }
    public void moveSlideToLength(double inches){

        slide.setSlideExtensionLength(inches);
    }
    public void moveElbowToAngle(double deg){

        elbow.setTargetAngle(deg);
    }

    public boolean isArmAtPresetPosition(ArmPresetPosition position){
        return Math.abs(elbow.getElbowAngle() - position.elbowAngle) < ArmConstants.ELBOWPRESETTOLERANCE &&  // angle is within 3 degrees of target
                Math.abs(slide.getSlideExtensionInInches() - position.slideLength) < ArmConstants.SLIDEPRESETTOLERANCE; // slide is within 1 inch of target
    }
    public double getSlideExtension(){

        return slide.getSlideExtensionInInches();
    }

    public HashMap getDebugInfo() {

        HashMap debugInfo = new HashMap<>();
        debugInfo.put("Slide Extension", this.getSlideExtension());
        debugInfo.put("Slide Limit", this.getMaximumSlideExtensionAllowedInInches());
        debugInfo.put("Slide Power", this.slide.slideMotor.getPower());
        debugInfo.put("Slide Current", this.slide.slideMotor.getCurrent(CurrentUnit.MILLIAMPS));
        debugInfo.put("Elbow Angle", this.getElbowAngleInDegrees());
        debugInfo.put("Elbow Power", this.elbow.elbowMotor.getPower());
        debugInfo.put("Elbow Current", this.elbow.elbowMotor.getCurrent(CurrentUnit.MILLIAMPS));
        return debugInfo;
    }


}
