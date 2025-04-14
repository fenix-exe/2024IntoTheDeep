package org.firstinspires.ftc.teamcode.teleop.stateModels;

public class StateModelParameters {

    public static class DriveStateParameters{
        public static double pitch = 0.5;
        public static double elbowAngle = 92;
        public static double slideLength = 8;
    }
    public static class IntakeStateParameters{
        public static double pitch=-90;
        public static double roll=-3;
        public static double downPitch=-90;
        public static double downRoll=-3;
        public static double elbowAngle=0;
        public static double slideLength=12;
    }
    public static class DepositStateParameters{
        public static double intermediatePitch = 0.5;
        public static double pitch = 100;
        public static double roll = 0;
        public static double elbowAngle = 92;
        public static double slideLength = 26;
        public static double slideRetractionLength = 8;
    }
    public static class DepositSampleIntoBucketStateParameters{
        public static double pitch = -105;
        public static double roll = -3;
        public static double elbowAngle = 80;
        public static double intermediateElbowAngle = 0;
        public static double slideLength = 12;
    }
    public static class EnterSubmersibleStateParameters{
        public static double pitch = 0.16;
        public static double waitTime = 100;
    }
    public static class LeaveSubmersibleStateParameters{
        public static double pitch = 0.36;
        public static double slideLength = 0;
    }
    public static class GrabBlockFromOutsideStateParameters{
        public static double downPitch = -105;
        public static double upPitch = 0;
        public static double upRoll = 0;
        public static double elbowIntakeDownAngleMediumSlides = -8;
        public static double elbowIntakeDownAngleFarSlides = -2;
        public static double elbowIntakeDownAngleCloseSlides = -12;
        public static double elbowIntakeUpAngle = 2;
        public static double waitTime = 500;
    }
    public static class PickupSpecimensStateParameters{
        public static double pitch = 0;
        public static double roll = -90;
        public static double elbowAngle = 0;
        public static double slideLength = 4.75;
        public static double elbowUpAngle = 31;
        public static double pickupSlideLength = 3;
        public static double endSlideLength = 9;
        public static double pitchEnd = 30;
        public static double rollEnd = -90;
    }
    public static class DepositSpecimenPositionStateParameters {
        public static double pitch = 0;
        public static double roll = -90;
        public static double elbowAngle = 28;
        public static double slideLength = 0;
    }
    public static class DropBlockAndMoveWristDown{
        public static double pitch = -105;
        public static double elbowAngle = 0.8;
        public static double slideLength;
    }
    public static class Hang{
        public static double pitch = 5;
        public static double linearActuatorExtension = 9.5;
        public static double intermediateLinearActuatorHeight = 8.5;
        public static double linearActuatorRetraction = 5.75;
        public static double slideExtensionToMoveElbow = 6;
        public static double initialElbowAngle = 65;
        public static double intermediateElbowAngle = 85;
        public static double hangElbowAngle = 95;
        public static double finalElbowAngle = 90;
        public static double slideExtension = 26;
        public static double slideIntermediatePosition = 12.5;
    }
    public static class DepositSampleIntoObservationZone{
        public static double retractionLength = 3;
        public static double pitchDown = 0;
        public static double extensionLength = 16;
        public static double downPitch = -105;
        public static double downRoll = -3;
    }

}
