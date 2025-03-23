package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.MecanumDrive;

import androidx.annotation.NonNull;

public class RobotWideFunctions {

    /*
    * This class allows for robotwidefunctions
     */


    /*
    * This action prints which step the robot is on on the driver station
     */
    public class vectorLog implements Action {
        private final double vector;
        private final Telemetry telemetry;

        vectorLog(double vector, Telemetry telemetry) {
            this.vector = vector;
            this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("vector complete:", vector);
            telemetry.update();
            telemetryPacket.put("vector complete:", vector);
            return false;
        }


    }
    public Action vectorLog(double vector, Telemetry telemetry) {
        return new vectorLog(vector, telemetry);
    }

    //This action changes the correction value in the MecanumDrive class
    // depending on how accurate we want the robot to be
    public class correctionChanger implements Action {
        private final double correction;

        correctionChanger(double correction) {
            this.correction = correction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            MecanumDrive.PARAMS.time_increase=correction;
            return false;
        }


    }
    public Action correctionChanger(double correction) {
        return new correctionChanger(correction);
    }
}
