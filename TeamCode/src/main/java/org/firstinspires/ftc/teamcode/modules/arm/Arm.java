package org.firstinspires.ftc.teamcode.modules.arm;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;

import java.util.HashMap;

public class Arm {
    Slide slide;
    public Elbow elbow;
    int physicalMaxExtension;
    double MaxSlideExtensionInches;

    public Arm(Slide slide, Elbow elbow, int initialTopHeight){
        this.slide = slide;
        this.elbow = elbow;
        this.physicalMaxExtension = initialTopHeight;
        ArmSpeedController.slide = slide;
    }

    public void moveSlide(double slideMovement, boolean remove_arm_rules) {
        int max_extension = getSlideMaxLengthIn42Inches(getElbowAngleInTicks());
        double power;
        if (!remove_arm_rules){
            if (slide.getSlideExtensionInInches() > max_extension - RobotConstants.SLIDE_TOLERANCE
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

    public int getSlideMaxLengthIn42Inches(int angleInTicks){
        int topHeight;
        double theta = elbow.ticksToDegrees(angleInTicks);
        if (theta != 90) {
            MaxSlideExtensionInches = ArmConstants.MAXSLIDEEXTENSIONLENGTHINCHES/(Math.cos(Math.toRadians(theta)));
        } else {
            MaxSlideExtensionInches = ArmConstants.SLIDEMAX;
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
        debugInfo.put("Slide Limit", this.getSlideMaxLengthIn42Inches(this.getElbowAngleInTicks()));
        debugInfo.put("Slide Power", this.slide.slideMotor.getPower());
        debugInfo.put("Slide Current", this.slide.slideMotor.getCurrent(CurrentUnit.MILLIAMPS));
        debugInfo.put("Elbow Angle", this.getElbowAngleInDegrees());
        debugInfo.put("Elbow Power", this.elbow.elbowMotor.getPower());
        debugInfo.put("Elbow Current", this.elbow.elbowMotor.getCurrent(CurrentUnit.MILLIAMPS));
        return debugInfo;
    }


}
