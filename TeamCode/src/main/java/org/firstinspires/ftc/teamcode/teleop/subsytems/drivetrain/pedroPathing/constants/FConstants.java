package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.9;

        FollowerConstants.xMovement = 74.93421844;
        FollowerConstants.yMovement = 59.36436528;

        FollowerConstants.forwardZeroPowerAcceleration = -25.9271;
        FollowerConstants.lateralZeroPowerAcceleration = -64.5358;

        //FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,0.05,0);
        //FollowerConstants.useSecondaryTranslationalPID = false;
        //FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        //FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.05,0);
        //FollowerConstants.useSecondaryHeadingPID = false;
        //FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0,0.0001,0.6,0);
        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.03,0,0.0000025,0,0.25);
        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1,0,0.00001,0,0);
        //FollowerConstants.useSecondaryDrivePID = false;
        //FollowerConstants.drivePIDFSwitch = 10;
        //FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(.0001,0,0,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 0.5;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 200;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 1;
        FollowerConstants.pathEndHeadingConstraint = 0.05;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
