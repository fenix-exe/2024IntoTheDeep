package org.firstinspires.ftc.teamcode.subsytems;

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
    public int getSlideMaxLength(int angleInTicks){
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
    public int currentAllowedMaxExtensionLength(){
        return getSlideMaxLength(elbow.getElbowTicks());
    }
    public boolean doesSlideNeedToRetract(int angleInTicks){
        return slide.getSlidePosition() > getSlideMaxLength(angleInTicks);
    }
}
