package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Red Back", group = "CenterStage")
public class CSAutoRedBack extends CSMethods {
    @Override
    public void runOpMode() {
        boolean isRed = true;
        setup(isRed);

        // ---------------------
        // ------Main Code------
        // ---------------------

        //dropCarWash();
        sleep(2000); // Wait to allow camera initialization (for detecting team prop)
        findPos();
        sleep(1000);
        drive(-20);
        sleep(1000);
        if (pos == 1) {
            turn(-30);
            sleep(1000);
            ejectPixel(); // Filler for placing pixel down the road
            sleep(1000);
            turn(30);
        } else if (pos == 2) {
            drive(-3);
            sleep(1000);
            ejectPixel();
            sleep(1000);
            drive(3);
        } else if (pos == 3) {
            turn(30);
            sleep(1000);
            ejectPixel();
            sleep(1000);
            turn(-30);
        }
        sleep(1000);
        turn(90);
        sleep(1000);
        drive(-10);
        sleep(2500);

        //*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }
}