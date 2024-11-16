package org.firstinspires.ftc.teamcode.subsytems.modules;

import org.firstinspires.ftc.teamcode.subsytems.pivot.pivotCodeFunctions;
import org.firstinspires.ftc.teamcode.subsytems.slides.slideCodeFunctions;

public class RobotArm {
    slideCodeFunctions slide;
    pivotCodeFunctions elbow;
    int physicalMaxExtension;
    int horizontalMaxLength;
    double MaxSlideExtensionInches;
    public RobotArm(slideCodeFunctions slide, pivotCodeFunctions elbow, int initialTopHeight, int horizontalMaxLength){
        this.slide = slide;
        this.elbow = elbow;
        this.physicalMaxExtension = initialTopHeight;
        this.horizontalMaxLength = horizontalMaxLength;
    }
    public int getSlideMaxLengthIn42Inches(int angleInTicks){
        int topHeight;
        double theta = elbow.ticksToDegrees(angleInTicks);
        if (theta != 90) {
            MaxSlideExtensionInches = 36/(Math.cos(Math.toRadians(theta)));
        } else {
            MaxSlideExtensionInches = 10^44;
        }
        int MaxSlideExtensionEncoderTicks = slide.InchesToTicks(MaxSlideExtensionInches);
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
    public double getSlideLengthInInches(){
        return slide.getSlideLengthInInches();
    }
}
