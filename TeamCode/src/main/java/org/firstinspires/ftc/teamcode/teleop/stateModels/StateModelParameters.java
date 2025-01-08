package org.firstinspires.ftc.teamcode.teleop.stateModels;

public class StateModelParameters {

    public static class DriveStateParameters{
        public static double pitch = 0;
        public static double elbowAngle = 58;
        public static double slideLength = 8;
    }
    public static class IntakeStateParameters{
        public static double pitch=0;
        public static double roll=-90;
        public static double downPitch=-90;
        public static double downRoll=0;
        public static double elbowAngle=12;
        public static double slideLength=12;
    }
    public static class DepositStateParameters{
        public static double pitch = -30;
        public static double roll = 0;
        public static double elbowAngle = 73;
        public static double slideLength = 30.5;
    }
    public static class DepositBackStateParameters{
        public static double pitch = 75;
        public static double roll = 0;
        public static double elbowAngle = 87;
        public static double slideLength = 24;
        public static double slideRetractionLength = 8;
    }
    public static class DepositSampleIntoBucketStateParameters{
        public static double pitch = 0;
        public static double roll = 0;
        public static double elbowAngle = 58;
        public static double slideLength = 8;
    }
    public static class GrabBlockFromOutsideStateParameters{
        public static double downPitch = -90;
        public static double upPitch = 0;
        public static double upRoll = 0;
        public static double elbowIntakeDownAngle = 4;
        public static double elbowIntakeUpAngle = 10;
        public static double elbowAngle = 58;
        public static double slideLength = 0;
    }
    public static class GrabBlockFromInsideStateParameters{
        public static double downPitch = -90;
        public static double upPitch = 0;
        public static double upRoll = -90;
        public static double elbowDownAngle = 2;
        public static double elbowUpAngle = 10;
        public static double elbowAngle = 58;
        public static double slideLength = 0;

    }
    public static class PickupSpecimensStateParameters{
        public static double pitch = -10;
        public static double roll = 90;
        public static double elbowAngle = 25;
        public static double slideLength = 0;
        public static double elbowUpAngle = 77;
        public static double endSlideLength = 3;
        public static double pitchEnd = 90;
        public static double rollEnd = 90;
    }
    public static class DepositSpecimensStateParameters{
        public static double pitch = 90;
        public static double roll = 90;
        public static double elbowAngle = 77;
        public static double elbowDownAngle = 58;
        public static double slideStartLength = 3;
        public static double slideDepositLength = 16;
    }
    public static class DropBlockAndMoveWristDown{
        public static double pitch = -90;
    }

}
