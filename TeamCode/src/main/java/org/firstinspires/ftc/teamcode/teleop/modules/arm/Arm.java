package org.firstinspires.ftc.teamcode.teleop.modules.arm;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.Slide;

import java.util.HashMap;

public class Arm {
    Slide slide;
    public Elbow elbow;

    public Arm(Slide slide, Elbow elbow){
        this.slide = slide;
        this.elbow = elbow;
    }

    public double moveSlide(double slideMovement, boolean remove_arm_rules) {
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
        return power;
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

    public double getMaximumSlideExtensionAllowedInInches(){
        double theta = elbow.getElbowAngle();
        double MaxSlideExtensionInches = RobotConstants.PHYSICAL_MAX_EXTENSION_IN_INCHES;
        if (!(Math.abs(90-theta) < 1)) { //tolerance of 1 degree around 90 degrees, I cannot compare double directly to int
            MaxSlideExtensionInches = Math.min(RobotConstants.PHYSICAL_MAX_EXTENSION_IN_INCHES,
                    Math.abs(ArmConstants.MAXSLIDEEXTENSIONLENGTHINCHES/(Math.cos(Math.toRadians(theta)))));
        }
        //add 1 inch safety margin
        MaxSlideExtensionInches -= RobotConstants.SLIDE_TOLERANCE;
        return MaxSlideExtensionInches;
    }
    public void resetSlideEncoders(){
        slide.resetEncoder();
    }
    public boolean isSlideTouchSensorPressed(){
        return slide.isHomingSwitchPressed();
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
        return slide.ticksToInches(slide.leftSlideMotor.getTargetPosition());
    }

    public void moveSlideToLength(double inches){

        slide.setSlideExtensionLength(inches);
    }
    public void moveElbowToAngle(double deg){

        elbow.setTargetAngle(deg);
    }
    public void setSlidePower(double power){
        slide.setSlidePower(power);
    }
    public boolean detectingMagneticLimitSwitch(){
        return elbow.isLimitSwitchPressed();
    }
    public double getSlideExtension(){

        return slide.getSlideExtensionInInches();
    }

    public HashMap getDebugInfo() {

        HashMap debugInfo = new HashMap<>();
        debugInfo.put("Slide Extension", String.valueOf(this.getSlideExtension()));
        debugInfo.put("Slide Limit", String.valueOf(this.getMaximumSlideExtensionAllowedInInches()));
        debugInfo.put("Slide Power", String.valueOf(this.slide.leftSlideMotor.getPower()));
        debugInfo.put("Slide Current", String.valueOf(this.slide.leftSlideMotor.getCurrent(CurrentUnit.MILLIAMPS)));
        debugInfo.put("Elbow Angle", String.valueOf(this.getElbowAngleInDegrees()));
        debugInfo.put("Elbow Power", String.valueOf(this.elbow.elbowMotor.getPower()));
        debugInfo.put("Elbow Current", String.valueOf(this.elbow.elbowMotor.getCurrent(CurrentUnit.MILLIAMPS)));
        return debugInfo;
    }


}
